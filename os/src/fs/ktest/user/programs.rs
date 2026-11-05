// 普通用户与文件行为保留原 inline suite，注册完整名仍属于 user::programs。
#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user-elf", "user-abi", "user-signal", "fs-path", "fs-oom"), resources("root"), timeout_ms = 90000)]
pub(crate) mod programs {
    use super::*;

    /// 原 musl 聚合程序在真实族根执行 argv/env，主映像与解释器身份从用户栈核对。
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

    /// 原 glibc PIE 聚合程序在真实族根执行 argv/env，退出与离栈仍由原拥有者消费。
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

    /// 原 loader 自身作为无 PT_INTERP 的 ET_DYN 入口启动，再由其正常执行原聚合程序。
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

    /// 两映像的非 VM fork 保留两份 inode 执行责任，真实末程序退出后才允许写准备。
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

    /// 解释器普通错误与真实 OOM 只销毁候选，旧两映像随后仍正常运行并完整退出。
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

    /// 普通只读日志验证独立 open、共享 dup、覆盖恢复和真实用户复制错误的消费点。
    #[test]
    #[ktest(tags("observation-user", "kmsg"))]
    fn observation_kmsg(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.observation_mode(context, b"kmsg")
    }

    /// 真实 ELF 的边缘、BSS、参数和缓存私有性由 loader 与实际用户指令共同验证。
    #[test]
    #[ktest(min_cpus = 2)]
    fn load_boundaries_stack_and_disk_program(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        layout::run(context, fixture)
    }

    /// 0644固定映像实际从页零或0x100进入，跨页程序头和未消费元数据不阻断执行。
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

    /// 大BSS和远端LOAD只按实际用户触页驻留，仍由原fixture消费退出与全部资源。
    #[test]
    #[ktest(tags("elf-direct-load"), min_cpus = 2)]
    fn direct_elf_sparse_bss(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        layout::direct_sparse(context, fixture)
    }

    /// 合法重叠LOAD的文件后写与BSS清零由用户指令及完整内存字节共同消费。
    #[test]
    #[ktest(tags("elf-direct-load"), min_cpus = 2)]
    fn direct_elf_overlapping_segments(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        layout::direct_overlap(context, fixture)
    }

    /// OOM、坏参数和未发表归还之后仍正常运行原真实程序。
    #[test]
    fn failed_preparation_keeps_original_program(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        preparation::run(context, fixture)
    }

    /// 用户指令核文件真实前缀/cookie、只读边界、映射和堆内容，退出仍由正常图回收。
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

    /// 两个真实用户进程共享管道OFD，读完最后writer的尾部后才见EOF。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_fork_dup_full_stream_tail_and_eof(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"d")
    }

    /// 两个writer把总长度4096的writev作为一笔请求提交，逐record核字节不交错。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_small_writev_records_across_writers(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"a")
    }

    /// NONBLOCK容量不足保EAGAIN，零长和跨页故障保已经真正消费的前缀。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_nonblocking_capacity_zero_and_fault_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"n")
    }

    /// 已解码iov只进入一次普通文件操作，长度边界与共享位置按实际字节推进。
    #[test]
    #[ktest(tags("fs-pipe", "fs-vector-user"))]
    fn file_vector_import_and_shared_position(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"v")
    }

    /// 无reader的非零write由写者线程领取SIGPIPE，忽略、默认和阻塞mask保持各自语义。
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

    /// 已经真正Blocked的read由普通信号中断，SA_RESTART仅由原用户返回协议决定重放。
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

    /// pipe2输出错误与第二槽不足只取消本次未发表预留，已有描述符与容量不改变。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe2_output_fault_and_second_slot_exhaustion(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"f")
    }

    /// 非root用户创建的匿名端点保留真实uid/gid，属性由正常fstat交付。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user"))]
    fn pipe_creator_credentials_in_anonymous_stat(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"u")
    }

    /// 普通pread/pwrite先按OFD位置能力返回ESPIPE，错误端点和地址不改变优先级。
    #[test]
    #[ktest(tags("fs-pipe", "fs-pipe-user", "fs-pipe-positioned"))]
    fn pipe_positioned_io_error_precedence(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.pipe_abi(context, b"p")
    }

    /// 目标指令核对 poll 数组、select 集合跨度、重复 fd 计数及实际错误顺序。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T10", "T12"))]
    fn poll_select_raw_abi_and_errors(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"a")
    }

    /// 零 fd 的有限等待消耗原绝对期限，普通结果不被剩余时间写回错误覆盖。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T09"))]
    fn zero_fd_deadlines(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.events_abi(context, b"t")
    }

    /// 三种等待各自进入真实 Blocked 后才让子进程写管道，通知通过生产源返回。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T11"))]
    fn persistent_event_blocked_notifications(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"b")
    }

    /// 真处理函数观察临时 mask，返回后旧 mask 恢复且等待不受 SA_RESTART 重启。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T09"))]
    fn temporary_mask_handler_restore(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"s")
    }

    /// raw 事件交付保留 data 高位，ADD、MOD 和 LT、ET、ONESHOT 消费同一真实管道。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T12", "T13", "T15"))]
    fn epoll_ready_modes(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.events_abi(context, b"e")
    }

    /// dup 登记与复用后的新 OFD 分别保持身份，末端点关闭同步解除原登记。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T14", "T18"))]
    fn epoll_dup_fd_reuse_final_close(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"r")
    }

    /// 首项 EFAULT 与跨页正前缀分别保留未交付候选，下次正常输出继续交付。
    #[test]
    #[ktest(tags("fs-events", "fs-events-user", "T17"))]
    fn epoll_output_fault_prefix(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.events_abi(context, b"f")
    }

    /// 普通 C 经真实 uname/chroot 系统调用核字节故障、旧 cwd/FD 和独立父子根。
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

    /// 原生 C 在可写卷核时间字段、特殊纳秒和权限，用户操作的实际错误不能由根 fixture 覆盖。
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

    /// 原生 C 从硬件 UTC 核日期字段和绝对睡眠，单调与 CPU 账保持各自语义。
    #[test]
    #[ktest(tags("runtime-clock", "user-libc", "fs-path"), timeout_ms = 120000)]
    fn runtime_clock(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.runtime_clock(context)
    }

    /// 根凭据从真实 C 程序消费限额、fork 与 exec 主栈范围，退出后沿原用户拥有者归还。
    #[test]
    #[ktest(tags("runtime-limits"), min_cpus = 1, timeout_ms = 120000)]
    fn runtime_stack_limits_root(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.runtime_limits(context, Credentials::ROOT)
    }

    /// 非根凭据从真实 C 程序核限额权限和 exec 范围，用户错误不被 fixture 根凭据覆盖。
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

    /// 普通静态 C 从真实磁盘启动，核 TLS、主栈、标准流和基础文件 ABI。
    #[test]
    #[ktest(tags("T31", "user-libc"), timeout_ms = 120000)]
    fn libc_static_smoke(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.libc_static_smoke(context)
    }

    /// 同一用户 Task 在两次 pipe 唤醒窗口实际迁核并校验 TLS 与浮点现场。
    #[test]
    #[ktest(min_cpus = 2, tags("T31", "user-libc"), timeout_ms = 120000)]
    fn libc_migration(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.libc_migration(context)
    }

    /// 普通脚本的解释器参数、环境和原始执行路径由实际用户程序核验。
    #[test]
    #[ktest(tags("T32", "user-script"), timeout_ms = 120000)]
    fn script_exec(context: &mut TestContext<'_>, fixture: &mut UserOwner) -> TestResult {
        fixture.script_exec(context)
    }

    /// 系统根以普通 fork、pipe 和 wait 消费两类孤儿后才请求真实 PowerOff 意图。
    #[test]
    #[ktest(tags("T33", "fs-system-user", "fs-pid1"), timeout_ms = 120000)]
    fn system_pid1_adopts_orphans(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.system_pid1_adopts_orphans(context)
    }

    /// 同号码内核根按普通 SIGKILL 退出，两种真实凭据分别核 reboot 权限与参数错误。
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

    /// 用户指令从 RO devfs 的 RDWR 设备请求共享写映射，能力拒绝先于挂载写检查。
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

    /// 正常盘面设备节点经 unlink 或 rename 覆盖结束链接，末使用归还实际inode位而不解释设备号为块树。
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

    /// 同一磁盘 ELF 的普通写入与同步后冷读取复用原 UserOwner、真实退出和最终资源比较。
    #[test]
    #[ktest(min_cpus = 2, tags("W35", "fs-writeback"))]
    fn production_writeback_sync_and_cold_reopen(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.production_writeback_cold_reopen(context)
    }

    /// 非owner用户指令核目录读DAC、逐组件search及拒绝后原游标和最低空槽。
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

    /// 四个代表路径覆盖私有fork、共享VM、独立组共享动作及线程组，不穷举共享轴。
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

    /// 另一真实Task保父fd0预留时rawfork，child普通open获得0且原OFD/flags关系不变。
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

    /// 普通/private/group合并、RT FIFO及真实队列耗尽由raw程序逐实例核对。
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

    /// 同一SIGHAND和VM共享者仍有各自进程组pending，clone后旧pending不继承。
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

    /// NODEFER、动作mask、RESETHAND和默认忽略均观察处理函数实际执行次数。
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

    /// clone已接受的零退出信号不误发SIGCHLD，普通SIGCHLD分支仍真实递送。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_clone_exit_notification_semantics(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.signal_clone_exit_notification(context)
    }

    /// 无handler停续后参数EFAULT恢复原mask，后续普通handler不消费残留saved_mask。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_sigsuspend_restart_restores_mask(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.signal_suspend_restart_mask(context)
    }

    /// 逐个 FP/向量寄存器、FCSR/FCC 和用户修改 PC/SP 经信号往返，LA 还必须经历真实用户 timer。
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

    /// 同步缺页、权限、整页EOF和非法指令核siginfo与发生时PC，用户改帧后实际续行。
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

    /// 子进程嵌套altstack后真实父wait，并逐字核父私有页未被内核frame写入污染。
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

    /// 读取坏sigreturn和写入无效栈均有限SIGSEGV，不把用户异常当框架panic。
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

    /// REAL中断相对/绝对sleep，waitid分别核SA_RESTART与EINTR，futex信号沿下一批。
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

    /// stop/continue后restart_syscall仍消费原期限，不把重启当作新相对请求。
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

    /// waitid的stopped/continued来自真实双成员组，父与独立共享VM组不混合。
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

    /// 两CPU间发送SIGKILL之前，原CPU必须timer抢占无限用户计算并推进另一个内核Task。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_timer_preempts_infinite_user_and_remote_kill(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.preempt_infinite(context)
    }

    /// 用户/内核CPU账、times的USER_HZ和CPU itimer不把offCPU睡眠算成执行。
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

    /// 不可中断原页加载先完成真实盘责任和用户read，再到返回点处理已经pending的信号。
    #[test]
    #[ktest(min_cpus = 2)]
    fn signal_wait_keeps_file_read_responsibility(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.signal_during_file_wait(context)
    }

    /// 不改用户字的真实唤醒、两种Mm隔离和共享匿名不同VA均由普通用户指令验证。
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

    /// 重排保原身份/期限，取消从当前位置消费一次，bitset配额只计被选择者。
    #[test]
    #[ktest(min_cpus = 2)]
    fn futex_bitset_requeue_signal_and_timeout(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.futex_program(context, b"r")
    }

    /// 用户以实际child_tid等待join；旧Mm清零唤醒与坏地址退出均完成正常回收。
    #[test]
    #[ktest(min_cpus = 2)]
    fn futex_clear_child_tid_and_bad_exit_address(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.futex_program(context, b"t")
    }

    /// SA_RESTART恢复原用户请求，停续后的相对等待保首次期限且不重读已解除参数页。
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

    /// 非leader沿原Task转移Linux身份；两个实际exec只允许唯一新映像或真实组fatal。
    #[test]
    #[ktest(min_cpus = 2)]
    fn exec_nonleader_and_concurrent_control(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_exec(context)
    }

    /// 外部共享files/sighand不被exec修改，旧根保活，新映像与CLOEXEC由普通程序验证。
    #[test]
    #[ktest(min_cpus = 2)]
    fn exec_shared_resources_and_mm_handoff(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_resources(context)
    }

    /// vfork按旧Mm使用结束恢复父；父被取消后子仍独立退出并完成转亲回收。
    #[test]
    #[ktest(min_cpus = 2)]
    fn vfork_exec_exit_and_parent_cancellation(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_vfork(context)
    }

    /// leader先退出保同组父资格；组退出、转亲和角色释放后才复用号码。
    #[test]
    #[ktest(min_cpus = 2)]
    fn exit_group_parentage_and_pid_role_reuse(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_roles(context)
    }

    /// 两个真实阻塞wait者可重复观察，但只能消费一次；坏copyout不恢复僵尸。
    #[test]
    #[ktest(min_cpus = 2)]
    fn wait_observation_consumption_and_copyout(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_wait(context)
    }

    /// 默认忽略仍保wait状态；显式忽略和NOCLDWAIT自动回收由普通用户syscall判定。
    #[test]
    #[ktest(min_cpus = 2)]
    fn sigchld_autoreap_and_wait_semantics(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare(context)?;
        fixture.lifecycle_sigchld(context)
    }

    /// 普通用户错误按真实wait状态和离栈join验证，内核或夹具错误仍归原框架。
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

    /// 远核read仍保有Mm和真实DMA请求时取消，完成、退出和离栈各自结束后才回收。
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
