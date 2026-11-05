// suite 保留在 user_execution 命名空间内，宏据此生成原注册全名与锚点。
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

    /// 用户实际 stack/FP 运算后两次 syscall 保持完整现场，原 Task 调度后仍回原 Rust 栈。
    #[test]
    fn user_round_trip(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (mm, context, direct) = image(0, 0)?;
        let fcsr = context.fcsr;
        fixture.start(0, user(mm, context)?, move |user, token, stop| {
            let local = [0x8167usize; 16];
            let stack = black_box(&local) as *const _ as usize;
            let id = task::current_task().id();
            let mut saved: Option<Registers> = None;
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
            // SAFETY: 本例 Mm 保活映像的完整数据页，唯一用户执行者已返回内核，
            // 代码全部 store 已在首个 syscall 前完成，不存在其他写者或 Rust 存储别名。
            check_probe(unsafe { probe_words(direct) }, 0, fcsr);
            Ok(42)
        })?;
        fixture.joined(0, 42)
    }

    /// 两个独立现场实际计算 FP/FCC后，在中断、重复进入和全部在线CPU迁核间保持完整寄存器。
    #[test]
    fn user_private_contexts(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        for slot in 0..2 {
            let (mm, context, direct) = image(arch::loop_offset(), slot)?;
            let fcsr = context.fcsr;
            fixture.start(slot, user(mm, context)?, move |user, token, stop| {
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
                // SAFETY: 本例唯一用户执行者已到不写数据的稳态分支，Mm 仍保活完整数据页，
                // 当前只在其原 Task 中读取已完成的 store，其他 CPU 没有该映像的写者。
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
                // fork 从真正执行过的完整现场派生，再让复制结果经过同一硬件进入/退出路径。
                let copied = user.context().fork();
                *user.context() = copied;
                // 时间片中断只发表调度请求；重进无 syscall 的用户循环前先完成原 Task 的安全点。
                sched::cond_resched();
                interrupted(&user, cpu::current().id);
                assert!(
                    Registers::read(&user) == saved,
                    "copied user registers changed"
                );

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

    /// 真实偶奇页读写及未映射、PROT_NONE、缓存只读modify和独立NR权限错误。
    #[test]
    fn user_permissions(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
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

    /// 真正从已驻留只读数据页取指，保留硬件拒绝后的原目标PC与访问方向。
    #[test]
    fn user_no_execute(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        permission_fault(fixture, Some(RO), UserAccess::Execute)
    }

    /// 同一正式用户入口验证当前权限，fault不推进PC，fixture仍等待原Task离栈。
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
        // LA只读modify先load填入TLB，未映射/PROT_NONE/纯执行页的store直接执行。
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

    /// 未准备中间页不发表槽，替换交回旧叶，解除末叶将表页保留到调用方完成失效后。
    #[test]
    fn user_table_ownership(_: &mut TestContext<'_>, _: &mut Users) -> TestResult {
        let mut table = UserPageTable::try_new().map_err(|_| TestError("NoMemory: table root"))?;
        let first = mm::alloc_pages(0, true).ok_or(TestError("NoMemory: first table leaf"))?;
        let second =
            mm::alloc_pages(0, true).ok_or(TestError("NoMemory: replacement table leaf"))?;
        let mut prepared = PreparedTables::default();
        // SAFETY: 本地表从未激活且由当前任务独占，两页完整清零、无用户或Rust排他访问；
        // 原页拥有保留到解除与失效后，prepared始终在所有软件修改结束后析构。
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
