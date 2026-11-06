// 用户态特权级入口、寄存器上下文保持与硬件页表（`UserPageTable`）生命周期回归测试套件。
//
// # 模块职责与架构定位
// 本套件由 `os/src/ktest/user_execution.rs` 通过 `include!` 内联引入，使 `#[ktest::suite]` 宏在同一模块路径下生成测试注册槽。
// - **上游调用者**：`ktest` 测试运行器在普通线程上下文中驱动本套件（测试夹具为 `super::fixture::Users`）。
// - **下游依赖**：`crate::arch`（用户态切换 `run_user` 与位置无关探测指令）、`crate::mm::user_table::UserPageTable`、
//   `crate::task::UserThread` 以及 `crate::sched` 调度与跨核迁移接口。
//
// # 核心设计与探测虚拟地址空间布局
// `probe::image` 为每个受测任务构建独立的 `Mm` 地址空间与初始寄存器上下文，其用户虚拟页布局如下：
//
// ```text
//   虚拟地址范围               权限   用途与测试角色
//   ─────────────────────────────────────────────────────────────────────────────────────────
//   0x0001_0000 (+4KiB CODE)   R+X    装载 arch::user_code() 位置无关探测机器码：
//                                       - 偏移 0         : 栈读写 + FP 运算 + 连续两次 syscall
//                                       - loop_offset    : 栈/FP/FCC/向量写入后进入原地自旋
//                                       - exit_offset    : 触发 SYS_exit(43)
//                                       - fault_offset   : 对寄存器 A5 指向地址执行读/写/取指
//   0x0002_0000 (+4KiB DATA)   R+W    保存用户态指令实际写回的 SP/TP/GP/FP/FCSR/FCC 与向量字
//   0x0003_0000 (+4KiB STACK)  R+W    用户态私有栈（初始 SP = STACK_TOP = 0x0003_1000）
//   0x0004_0000 (+4KiB TARGET) 可变   按用例需求映射偶/奇读写页、RO/EX/NONE 页或保持未映射
// ```
//
// # 锁获取顺序与并发契约
// - 每个受测 `UserThread` 由单一测试子任务独占驱动，`UserThread::context()` 通过内部 `RefCell` 提供短期可变借用；
//   在调用 `event`、`interrupted`、`sched::yield_now` 或 `sched::test_migrate_to` 前必须释放 `RefMut` 借用。
// - 直接通过内核线性映射指针（`direct`）读取 `DATA` 页前，必须确认唯一的用户态执行者已陷入返回内核态（或已进入不写内存的自旋分支），
//   防止与用户态指令产生并发读写数据竞争。
#[::ktest::suite(fixture = Users, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("user-entry"), timeout_ms = 15000)]
pub(crate) mod entry {
    use core::hint::black_box;
    use core::sync::atomic::Ordering;

    use super::fixture::Users;
    #[cfg(target_arch = "loongarch64")]
    use super::probe::DATA;
    use super::probe::{
        check_probe, event, image, interrupted, page, probe_words, user, Registers, A0, A5, A7,
        CODE, EX, NONE, RO, RW, T0, TARGET,
    };
    #[cfg(target_arch = "loongarch64")]
    use crate::arch::user_fp_lanes;
    use crate::arch::{self, UserAccess, UserTrap};
    use crate::ktest::execution_core::until;
    use crate::mm::user_table::{PagePermissions, PageTableError, PreparedTables, UserPageTable};
    use crate::{cpu, mm, sched, task};
    use ::ktest::{TestContext, TestError, TestResult};

    /// 验证用户态栈操作与浮点运算后连续两次系统调用的上下文完整性，以及任务让出 CPU 后内核栈帧的保持。
    ///
    /// - **测试目标**：确认用户态执行栈读写与浮点加法后通过 `syscall` 陷入内核时，通用与浮点寄存器完整保存，
    ///   手动推进 `PC += 4` 并经历 `sched::yield_now()` 调度切换后再次进入用户态仍能准确执行下一条系统调用，且内核局部栈帧不被篡改。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 构建默认入口（偏移 `0`）的用户地址空间与 `UserThread`，在子任务栈上放置哨兵数组 `local`。
    ///   2. 连续两次进入用户态捕获 `UserTrap::Syscall`，分别校验系统调用号 `A7 == 93` 与参数 `A0 == 41, 42`，每次将 `PC` 前推 4 字节并调用 `yield_now()`。
    ///   3. 对比两次系统调用之间的全套寄存器快照，并在返回内核后校验 `DATA` 页中由用户态指令写回的计算结果。
    /// - **预期结果**：两次系统调用除 `A0` 按指令更新外其余寄存器完全一致，子任务 ID 与内核栈哨兵保持不变，`DATA` 页探测结果匹配预期。
    #[test]
    fn user_round_trip(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (mm, context, direct) = image(0, 0)?;
        let fcsr = context.fcsr;
        fixture.start(0, user(mm, context)?, move |user, token, stop| {
            // Step 1: 在当前测试子任务的内核 Rust 栈上布置哨兵数组，用于验证用户态切换不会破坏内核栈。
            let local = [0x8167usize; 16];
            let stack = black_box(&local) as *const _ as usize;
            let id = task::current_task().id();
            let mut saved: Option<Registers> = None;
            // Step 2: 依次捕获用户态连续发起的两次系统调用（A0 = 41 与 A0 = 42），每次推进 PC 并主动让出 CPU。
            for expected in [41, 42] {
                assert_eq!(event(&user, &token, &stop)?, UserTrap::Syscall);
                {
                    let mut context = user.context();
                    assert_eq!(context.gpr[A7], 93);
                    assert_eq!(context.gpr[A0], expected);
                    context.pc = context.pc.checked_add(4).expect("user syscall PC overflow");
                }
                assert_eq!(task::current_task().id(), id);
                assert_eq!(black_box(&local) as *const _ as usize, stack);
                assert!(black_box(&local).iter().all(|word| *word == 0x8167));
                if let Some(saved) = &mut saved {
                    saved.gpr[A0] = expected;
                    assert!(
                        *saved == Registers::read(&user),
                        "user registers changed across syscalls"
                    );
                } else {
                    saved = Some(Registers::read(&user));
                }
                sched::yield_now();
            }
            // Step 3: 校验用户态在首次系统调用前写入 DATA 物理页的栈指针、TLS/GP 与浮点运算结果。
            // SAFETY:
            // 1. `direct` 指向的 `DATA` 物理页由当前 `user` 持有的 `Mm` 持续保活，且已在 `probe::page` 中整页清零初始化；
            // 2. 本用例唯一的用户态执行者已在第二次系统调用后返回当前内核任务，探测代码的所有内存写入均在第一次系统调用前完成，无并发写者或别名冲突。
            check_probe(unsafe { probe_words(direct) }, 0, fcsr);
            Ok(42)
        })?;
        fixture.joined(0, 42)
    }

    /// 验证两个并发用户态上下文在时钟中断抢占、`fork` 上下文克隆及遍历全部在线 CPU 迁移过程中的寄存器隔离与保持。
    ///
    /// - **测试目标**：确认多个独立 `UserThread` 的通用寄存器、浮点标量/向量通道（LoongArch LSX/LASX）及浮点条件码（FCC/FCSR）
    ///   在异步中断返回、`UserContext::fork()` 派生和跨核迁移后互不串扰、零丢失。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 并发启动槽位 `0` 和 `1` 两个子任务（各自使用不同槽位特征初值），驱动用户态执行完计算并停留在无内存写入的 `loop_spin` 自旋指令处。
    ///   2. 读取并校验 `DATA` 页写回结果（在 LoongArch64 下额外校验 `FCC` 位域与全部浮点向量通道写回值）。
    ///   3. 调用 `user.context().fork()` 克隆当前上下文并写回，触发一次中断往返后比对寄存器快照。
    ///   4. 依次将任务迁移至系统中每一个在线 CPU（循环两轮），每次在目标核上经历用户态中断返回并校验全套寄存器与 `PC`。
    ///   5. 将 `PC` 指向 `exit_offset` 触发退出系统调用，校验最终浮点寄存器状态。
    /// - **预期结果**：两个并发任务均以状态码 `43` 正常退出，全过程中通用、浮点、向量及条件码寄存器与初始稳态快照完全一致。
    #[test]
    fn user_private_contexts(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        for slot in 0..2 {
            let (mm, context, direct) = image(arch::loop_offset(), slot)?;
            let fcsr = context.fcsr;
            fixture.start(slot, user(mm, context)?, move |user, token, stop| {
                // Step 1: 反复进入用户态直至异步时钟中断在 `loop_spin_offset` 自旋点处捕获该任务。
                until(
                    || {
                        if stop.load(Ordering::Acquire) || token.stop_requested() {
                            return true;
                        }
                        let current_cpu = cpu::current().id;
                        interrupted(&user, current_cpu);
                        user.context().pc == CODE + arch::loop_spin_offset()
                    },
                    "user probe did not enter its steady instruction loop",
                )?;
                if stop.load(Ordering::Acquire) || token.stop_requested() {
                    return Ok(0);
                }
                // Step 2: 校验用户态进入自旋前写回 DATA 页的计算结果与架构专属向量/条件码寄存器。
                // SAFETY:
                // 1. `direct` 对应的 `DATA` 物理页由本任务独占持有的 `Mm` 保活且已完成整页初始化；
                // 2. 上方 `until` 已确认唯一的用户态执行者停留在只含无条件跳转指令的 `loop_spin_offset` 处，后续不再写入 `DATA` 页，无并发写者。
                let words = unsafe { probe_words(direct) };
                check_probe(words, slot, fcsr);
                #[cfg(target_arch = "riscv64")]
                assert_eq!(words[7], 1);
                #[cfg(target_arch = "loongarch64")]
                {
                    assert_eq!(words[6], 1);
                    assert_eq!(words[7], 0);
                    let fcc = user.context().fcc;
                    assert_eq!((fcc >> 24) & 0xff, 1);
                    assert_eq!((fcc >> 56) & 0xff, 0);
                    let context = user.context();
                    for lane in 0..user_fp_lanes() {
                        let expected = 0x8190 + (slot * 4 + lane) as u64;
                        assert_eq!(context.fpr[31].lanes[lane], expected);
                        let mut bytes = [0u8; 8];
                        let progress = user.mm().unwrap().copyin(&mut bytes, DATA + 128 + lane * 8);
                        assert_eq!((progress.copied, progress.error), (8, None));
                        assert_eq!(u64::from_le_bytes(bytes), expected);
                    }
                }

                let saved = Registers::read(&user);
                // Step 3: 验证 `UserContext::fork()` 克隆出的寄存器现场经过硬件进入/退出路径后保持原值。
                let copied = user.context().fork();
                *user.context() = copied;
                // 时钟中断仅置位调度重选标志；在重新进入无系统调用的用户态自旋循环前，先在内核安全点响应挂起的调度请求。
                sched::cond_resched();
                interrupted(&user, cpu::current().id);
                assert!(
                    Registers::read(&user) == saved,
                    "copied user registers changed"
                );

                // Step 4: 将当前任务依次迁移到所有在线 CPU 核心（连续两轮），验证跨核切换与中断返回不破坏用户寄存器。
                let id = task::current_task().id();
                for target in (0..cpu::count()).chain(0..cpu::count()) {
                    if stop.load(Ordering::Acquire) || token.stop_requested() {
                        return Ok(0);
                    }
                    sched::test_migrate_to(target);
                    assert_eq!(cpu::current().id, target);
                    assert_eq!(task::current_task().id(), id);
                    interrupted(&user, target);
                    assert!(
                        Registers::read(&user) == saved,
                        "user registers changed across interruption/migration"
                    );
                    assert_eq!(user.context().pc, CODE + arch::loop_spin_offset());
                    sched::cond_resched();
                }

                // Step 5: 将用户 PC 改写至退出系统调用入口，验证系统调用陷入路径同样完整保留浮点寄存器通道。
                user.context().pc = CODE + arch::exit_offset();
                assert_eq!(event(&user, &token, &stop)?, UserTrap::Syscall);
                assert_eq!(user.context().gpr[A0], 43);
                assert_eq!(
                    user.context().fpr,
                    saved.fpr,
                    "user FP lanes changed across syscall"
                );
                Ok(43)
            })?;
        }
        fixture.joined(0, 43)?;
        fixture.joined(1, 43)
    }

    /// 验证已映射奇偶虚拟页的正常读写，以及未映射、`PROT_NONE`、只读页写入与纯执行页读写触发的页故障（Page Fault）。
    ///
    /// - **测试目标**：确认硬件 MMU 与陷入处理层能够正确放行合法的偶页/奇页读写，并准确拦截各类违反页表权限的内存访问。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 在 `TARGET`（偶页）与 `TARGET + PAGE_SIZE`（奇页）分别映射具有不同初始特征值的 `RW` 页，依次令用户态读取、清零写入再重读。
    ///   2. 枚举未映射（`None`）、无权限（`NONE`）、只读（`RO`）和纯执行（`EX`）四类目标页状态，对不允许的访问方向逐一调用 `permission_fault`。
    /// - **预期结果**：合法读写准确读出初值并在写入后变为 `0`；所有越权访问均精确上报对应的 `UserTrap::PageFault` 且不错误推进故障指令 `PC`。
    #[test]
    fn user_permissions(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        // Step 1: 分别验证偶页（TARGET）与奇页（TARGET + PAGE_SIZE）的用户态读取、写入清零与重读闭环。
        for odd in [false, true] {
            let (mut mm, mut context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
            page(&mut mm, TARGET, RW, &0x9851u64.to_ne_bytes())?;
            page(
                &mut mm,
                TARGET + mm::PAGE_SIZE,
                RW,
                &0x9852u64.to_ne_bytes(),
            )?;
            context.gpr[A5] = TARGET + usize::from(odd) * mm::PAGE_SIZE;
            fixture.start(0, user(mm, context)?, move |user, token, stop| {
                assert_eq!(event(&user, &token, &stop)?, UserTrap::Syscall);
                assert_eq!(user.context().gpr[T0], if odd { 0x9852 } else { 0x9851 });
                user.context().pc = CODE + arch::fault_offset(UserAccess::Write);
                assert_eq!(event(&user, &token, &stop)?, UserTrap::Syscall);
                user.context().pc = CODE + arch::fault_offset(UserAccess::Read);
                assert_eq!(event(&user, &token, &stop)?, UserTrap::Syscall);
                assert_eq!(user.context().gpr[T0], 0);
                Ok(43)
            })?;
            fixture.joined(0, 43)?;
        }

        // Step 2: 遍历未映射、PROT_NONE、只读页写访问与纯执行页读/写访问，逐一验证权限异常上报。
        for (permissions, accesses) in [
            (
                None,
                &[UserAccess::Read, UserAccess::Write, UserAccess::Execute][..],
            ),
            (
                Some(NONE),
                &[UserAccess::Read, UserAccess::Write, UserAccess::Execute][..],
            ),
            (Some(RO), &[UserAccess::Write][..]),
            (Some(EX), &[UserAccess::Read, UserAccess::Write][..]),
        ] {
            for &access in accesses {
                permission_fault(fixture, permissions, access)?;
            }
        }
        Ok(())
    }

    /// 验证从已映射的只读（不可执行，`RO`）数据页取指时，硬件触发取指页故障并保留目标故障 `PC`。
    ///
    /// - **测试目标**：确认页表不可执行（NX / 非可执行位）保护生效，用户态跳转至只读数据页不会执行其中的字节。
    /// - **场景步骤（Given-When-Then）**：在 `TARGET` 处映射 `RO` 物理页，令用户态通过寄存器间接跳转至 `TARGET` 取指。
    /// - **预期结果**：陷入返回 `UserTrap::PageFault { address: TARGET, access: UserAccess::Execute }`，且保存的用户 `PC` 停留在 `TARGET`。
    #[test]
    fn user_no_execute(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        permission_fault(fixture, Some(RO), UserAccess::Execute)
    }

    /// 构造指定页权限与访问方向的单页故障场景，断言内核准确捕获 `UserTrap::PageFault` 且故障指令 `PC` 未越过出错指令。
    ///
    /// 在普通可睡眠任务上下文中调用；子任务验证完成后正常返回并通过 `fixture.joined` 确认完全退出内核栈。
    fn permission_fault(
        fixture: &mut Users,
        permissions: Option<PagePermissions>,
        access: UserAccess,
    ) -> TestResult {
        let (mut mm, mut context, _) = image(arch::fault_offset(access), 0)?;
        if let Some(permissions) = permissions {
            page(&mut mm, TARGET, permissions, &[])?;
        }
        context.gpr[A5] = TARGET;
        // LoongArch64 的 `la_user_fault_write` 首条指令为 `ld.d`（用于先把只读页装入 TLB 以触发页修改异常 PME），第二条才是 `st.d`；
        // 当目标页未映射、`PROT_NONE` 或仅可执行（不可读）时，跳过首条 `ld.d`（+4 字节）直接执行 `st.d` 以测试纯写入故障。
        #[cfg(target_arch = "loongarch64")]
        if access == UserAccess::Write && permissions != Some(RO) {
            context.pc += 4;
        }
        let fault_pc = context.pc;
        fixture.start(0, user(mm, context)?, move |user, token, stop| {
            assert_eq!(
                event(&user, &token, &stop)?,
                UserTrap::PageFault {
                    address: TARGET,
                    access
                },
                "permissions={:?} access={:?} fault_pc={:#x}",
                permissions,
                access,
                fault_pc
            );
            let pc = user.context().pc;
            if access == UserAccess::Execute {
                assert_eq!(pc, TARGET);
            } else {
                // LoongArch64 只读页写入场景先成功执行了首条 `ld.d`（+4 字节），随后在第二条 `st.d` 处触发页修改异常。
                #[cfg(target_arch = "loongarch64")]
                let fault_pc = if access == UserAccess::Write && permissions == Some(RO) {
                    fault_pc + 4
                } else {
                    fault_pc
                };
                assert_eq!(pc, fault_pc);
            }
            Ok(19)
        })?;
        fixture.joined(0, 19)
    }

    /// 验证 `UserPageTable` 在缺少预分配目录页时的无副作用失败、叶子页替换旧映射返回，以及解映射后空目录页的延迟回收生命周期。
    ///
    /// - **测试目标**：确认 `map` 在 `PreparedTables` 为空时返回 `MissingTables` 且不残留半成品表项；
    ///   覆盖映射时准确交回旧叶子物理页地址；`unmap` 摘除最后一个叶子项后将空中间目录页放入 `RetiredTables`，供调用方刷新 TLB 后再安全释放。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 新建独占的 `UserPageTable` 与两个清零物理数据页 `first`、`second`，先传入空的 `PreparedTables` 尝试映射 `TARGET`。
    ///   2. 按 `needed_tables(TARGET)` 预分配中间目录页，首次映射 `first`（`RW`），随后在同一虚拟地址覆盖映射 `second`（`RO`）并带页内偏移查询。
    ///   3. 调用 `unmap(TARGET)` 解除映射，检查返回的旧映射与待建目录层数，执行 `flush_user_translations()` 后再释放 `retired` 目录页与数据页。
    /// - **预期结果**：空准备集映射返回 `Err(PageTableError::MissingTables)`；覆盖与解映射分别返回 `first` 与 `second` 的物理地址，
    ///   解映射后空中间目录页全部摘除（重新需要 `USER_LEVELS - 1` 级目录），且所有页资源在测试结束时完全归还。
    #[test]
    fn user_table_ownership(_: &mut TestContext<'_>, _: &mut Users) -> TestResult {
        let mut table = UserPageTable::try_new().map_err(|_| TestError("NoMemory: table root"))?;
        let first = mm::alloc_pages(0, true).ok_or(TestError("NoMemory: first table leaf"))?;
        let second =
            mm::alloc_pages(0, true).ok_or(TestError("NoMemory: replacement table leaf"))?;
        let mut prepared = PreparedTables::default();
        // SAFETY:
        // 1. `table` 是当前栈帧新建且从未装入任何 CPU 页表基址寄存器的私有 `UserPageTable`，由当前任务独占修改，无并发访问；
        // 2. `first` 与 `second` 均为刚刚分配并整页清零的独立物理页，且一直保活至 `unmap` 与 `flush_user_translations()` 完成之后才 `drop`；
        // 3. `retired` 持有的已摘除中间目录页在完成 `flush_user_translations()` 后才析构归还，满足页表生命周期契约。
        unsafe {
            assert_eq!(
                table.map(TARGET, first.phys_addr(), RW, &mut prepared),
                Err(PageTableError::MissingTables)
            );
            assert!(table.lookup(TARGET).is_none());

            prepared = PreparedTables::try_new(table.needed_tables(TARGET))
                .map_err(|_| TestError("NoMemory: table middle pages"))?;
            assert!(table
                .map(TARGET, first.phys_addr(), RW, &mut prepared)
                .unwrap()
                .is_none());

            let old = table
                .map(TARGET, second.phys_addr(), RO, &mut prepared)
                .unwrap()
                .unwrap();
            assert_eq!(old.physical, first.phys_addr());
            assert_eq!(
                table.lookup(TARGET + 17).unwrap().physical,
                second.phys_addr()
            );

            let (old, retired) = table.unmap(TARGET);
            assert_eq!(old.unwrap().physical, second.phys_addr());
            assert!(table.lookup(TARGET).is_none());
            assert_eq!(table.needed_tables(TARGET), arch::USER_LEVELS - 1);
            arch::flush_user_translations();
            drop(retired);
        }
        drop(prepared);
        drop(table);
        drop(first);
        drop(second);
        Ok(())
    }
}
