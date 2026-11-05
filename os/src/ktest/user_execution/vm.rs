// suite 保留在 user_execution 命名空间内，远核活动根与页表断言仍由原用例执行。
#[::ktest::suite(fixture = Users, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("user-vm"), timeout_ms = 15000)]
pub(crate) mod vm {
    use alloc::sync::Arc;
    use core::sync::atomic::{AtomicBool, Ordering};

    use super::fixture::{Controls, Users};
    use super::probe::{access, event, image, user, word, A5, CODE, RO, RW, STACK_TOP, T0, TARGET};
    use crate::arch::{self, UserAccess, UserContext, UserTrap};
    use crate::ktest::execution_core::{baseline, until};
    use crate::ktest::ChildToken;
    use crate::mm::space::VmError;
    use crate::mm::uaccess::CStringError;
    use crate::task::UserThread;
    use crate::{cpu, mm, sched, task};
    use ::ktest::{TestContext, TestError, TestResult};

    /// 首次实际用户 load/store 才安装匿名叶，相邻页及跨页复制保持完整零填语义。
    #[test]
    fn anonymous_first_touch(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (mm, context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        mm.map_anonymous(TARGET, 3 * mm::PAGE_SIZE, RW, RW).unwrap();
        assert!(mm.mapping(TARGET).is_none());
        assert!(mm.mapping(TARGET + mm::PAGE_SIZE).is_none());
        assert!(mm.mapping(TARGET + 2 * mm::PAGE_SIZE).is_none());
        fixture.start(0, user(mm, context)?, move |user, token, stop| {
            let mm = user.mm().unwrap();
            for address in [TARGET, TARGET + mm::PAGE_SIZE] {
                assert_eq!(
                    access(&user, &token, &stop, address, UserAccess::Read)?,
                    (1, 0)
                );
                let progress = mm.copyout(address, &0x9851u64.to_le_bytes());
                assert_eq!((progress.copied, progress.error), (8, None));
                assert_eq!(
                    access(&user, &token, &stop, address, UserAccess::Write)?.0,
                    0
                );
                assert_eq!(
                    access(&user, &token, &stop, address, UserAccess::Read)?.1,
                    0
                );
            }

            assert_eq!(
                access(
                    &user,
                    &token,
                    &stop,
                    TARGET + 2 * mm::PAGE_SIZE,
                    UserAccess::Write
                )?
                .0,
                1
            );
            assert_eq!(
                access(
                    &user,
                    &token,
                    &stop,
                    TARGET + 2 * mm::PAGE_SIZE,
                    UserAccess::Read
                )?
                .1,
                0
            );
            let mut crossing = [0xa5; 32];
            let progress = mm.copyin(&mut crossing, TARGET + mm::PAGE_SIZE - 16);
            assert_eq!((progress.copied, progress.error), (32, None));
            assert_eq!(crossing, [0; 32]);

            mm.unmap(TARGET, 3 * mm::PAGE_SIZE).unwrap();
            user.context().pc = CODE + arch::fault_offset(UserAccess::Read);
            user.context().gpr[A5] = TARGET;
            assert_eq!(
                event(&user, &token, &stop)?,
                UserTrap::PageFault {
                    address: TARGET,
                    access: UserAccess::Read,
                }
            );
            assert_eq!(mm.fault(TARGET, UserAccess::Read), Err(VmError::Unmapped));
            Ok(43)
        })?;
        fixture.joined(0, 43)
    }

    /// 私有 fork 后 copyout 与真实用户 store 分别拆页，父子内容互不污染且未写页继续共享。
    #[test]
    fn fork_copyout_and_user_store(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (parent, context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        parent
            .map_anonymous(TARGET, 2 * mm::PAGE_SIZE, RW, RW)
            .unwrap();
        for address in [TARGET, TARGET + mm::PAGE_SIZE] {
            let copied = parent.copyout(address, &0x9851u64.to_le_bytes());
            assert_eq!((copied.copied, copied.error), (8, None));
        }

        let child = parent.fork().unwrap();
        assert_eq!(
            parent.mapping(TARGET).unwrap().physical,
            child.mapping(TARGET).unwrap().physical
        );
        assert!(!parent.mapping(TARGET).unwrap().permissions.write);
        assert!(!child.mapping(TARGET).unwrap().permissions.write);

        let copied = child.copyout(TARGET, &0x9852u64.to_le_bytes());
        assert_eq!((copied.copied, copied.error), (8, None));
        assert_eq!(word(&parent, TARGET), 0x9851);
        assert_eq!(word(&child, TARGET), 0x9852);
        assert_ne!(
            parent.mapping(TARGET).unwrap().physical,
            child.mapping(TARGET).unwrap().physical
        );
        assert_eq!(
            parent.mapping(TARGET + mm::PAGE_SIZE).unwrap().physical,
            child.mapping(TARGET + mm::PAGE_SIZE).unwrap().physical
        );

        let parent = Arc::try_new(parent).map_err(|_| TestError("NoMemory: COW parent"))?;
        let child = Arc::try_new(child).map_err(|_| TestError("NoMemory: COW child"))?;
        let thread = UserThread::try_new(Arc::clone(&parent), context)
            .map_err(|_| TestError("NoMemory: COW user"))?;
        let child_check = Arc::clone(&child);
        fixture.start(0, thread, move |user, token, stop| {
            assert_eq!(
                access(&user, &token, &stop, TARGET, UserAccess::Write)?.0,
                1
            );
            assert_eq!(access(&user, &token, &stop, TARGET, UserAccess::Read)?.1, 0);
            assert_eq!(word(&child_check, TARGET), 0x9852);
            assert_eq!(word(&child_check, TARGET + mm::PAGE_SIZE), 0x9851);
            Ok(43)
        })?;
        fixture.joined(0, 43)?;
        assert_eq!(word(&parent, TARGET), 0);
        assert_eq!(word(&child, TARGET), 0x9852);
        drop(child);
        drop(parent);
        Ok(())
    }

    /// 只读但允许未来写的页在降升权后仍走 COW，两份同时故障和同 Mm 竞争都重新核对叶。
    #[test]
    #[ktest(min_cpus = 2)]
    fn readonly_future_write_and_racing_faults(
        _: &mut TestContext<'_>,
        fixture: &mut Users,
    ) -> TestResult {
        for same_mm in [false, true] {
            let controls = Controls::new(fixture)?;
            let (parent, _, _) = image(arch::fault_offset(UserAccess::Write), 0)?;
            parent.map_anonymous(TARGET, mm::PAGE_SIZE, RW, RW).unwrap();
            assert_eq!(parent.copyout(TARGET, &0x9851u64.to_le_bytes()).error, None);
            parent.protect(TARGET, mm::PAGE_SIZE, RO).unwrap();
            let child = parent.fork().unwrap();
            let untouched =
                Arc::try_new(child).map_err(|_| TestError("NoMemory: readonly child"))?;
            let parent =
                Arc::try_new(parent).map_err(|_| TestError("NoMemory: readonly parent"))?;
            let shared = parent.mapping(TARGET).unwrap().physical;
            assert_eq!(shared, untouched.mapping(TARGET).unwrap().physical);
            parent.protect(TARGET, mm::PAGE_SIZE, RW).unwrap();
            parent.protect(TARGET, mm::PAGE_SIZE, RO).unwrap();
            parent.protect(TARGET, mm::PAGE_SIZE, RW).unwrap();
            if !same_mm {
                untouched.protect(TARGET, mm::PAGE_SIZE, RW).unwrap();
            }

            for slot in 0..2 {
                let owner = if slot == 0 || same_mm {
                    Arc::clone(&parent)
                } else {
                    Arc::clone(&untouched)
                };
                let mut context =
                    UserContext::new(CODE + arch::fault_offset(UserAccess::Write), STACK_TOP);
                context.gpr[A5] = TARGET;
                let thread = UserThread::try_new(owner, context)
                    .map_err(|_| TestError("NoMemory: racing COW user"))?;
                let controls = Arc::clone(&controls);
                fixture.start(slot, thread, move |user, token, stop| {
                    sched::test_migrate_to(slot);
                    assert_eq!(
                        event(&user, &token, &stop)?,
                        UserTrap::PageFault {
                            address: TARGET,
                            access: UserAccess::Write,
                        }
                    );
                    let pc = user.context().pc;
                    controls.ready[slot].store(true, Ordering::Release);
                    controls.gates[slot].wait();
                    if stop.load(Ordering::Acquire) || token.stop_requested() {
                        return Ok(0);
                    }
                    sched::test_migrate_to(slot);
                    let mm = user.mm().unwrap();
                    mm.fault(TARGET, UserAccess::Write).unwrap();
                    assert_eq!(user.context().pc, pc);
                    assert_eq!(event(&user, &token, &stop)?, UserTrap::Syscall);
                    assert_eq!(access(&user, &token, &stop, TARGET, UserAccess::Read)?.1, 0);
                    Ok(43)
                })?;
            }

            until(
                || {
                    controls.ready[..2]
                        .iter()
                        .all(|ready| ready.load(Ordering::Acquire))
                },
                "COW participants did not fault before release",
            )?;
            controls.gates[0].complete();
            controls.gates[1].complete();
            fixture.joined(0, 43)?;
            fixture.joined(1, 43)?;

            assert_eq!(word(&parent, TARGET), 0);
            assert_ne!(parent.mapping(TARGET).unwrap().physical, shared);
            assert_eq!(word(&untouched, TARGET), if same_mm { 0x9851 } else { 0 });
            if !same_mm {
                assert_ne!(
                    parent.mapping(TARGET).unwrap().physical,
                    untouched.mapping(TARGET).unwrap().physical
                );
            }
            drop(untouched);
            drop(parent);
        }
        Ok(())
    }

    /// 真实跨页访问保留失败前缀，字符串保留原始字节，原子独立访问遵守对齐与长度边界。
    #[test]
    fn bounded_uaccess(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (mm, context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        mm.map_anonymous(TARGET, 2 * mm::PAGE_SIZE, RW, RW).unwrap();
        let high = arch::USER_ADDRESS_END - mm::PAGE_SIZE;
        mm.map_anonymous(high, mm::PAGE_SIZE, RW, RW).unwrap();
        fixture.start(0, user(mm, context)?, move |user, token, stop| {
            let mm = user.mm().unwrap();
            let boundary = TARGET + mm::PAGE_SIZE;
            for offset in 0..8 {
                for length in 0..17 {
                    let address = boundary - 8 + offset;
                    let source = core::array::from_fn::<_, 24, _>(|index| (index * 13 + 7) as u8);
                    let copied = mm.copyout(address, &source[..length]);
                    assert_eq!((copied.copied, copied.error), (length, None));
                    let mut result = [0xa5; 24];
                    let copied = mm.copyin(&mut result[..length], address);
                    assert_eq!((copied.copied, copied.error), (length, None));
                    assert_eq!(&result[..length], &source[..length]);
                    assert_eq!(&result[length..], &[0xa5; 24][length..]);
                }
            }

            let number = 0x0102030405060708u64;
            assert_eq!(mm.copyout(boundary - 3, &number.to_le_bytes()).error, None);
            assert_eq!(mm.read_u64(boundary - 3), Ok(number));
            assert_eq!(mm.read_u32(boundary - 3), Ok(0x05060708));
            assert_eq!(mm.read_usize(boundary - 3), Ok(number as usize));

            assert_eq!(mm.copyout(boundary - 2, &[0xff, 0xfe, 0x80, 0]).error, None);
            assert_eq!(
                mm.read_c_string(boundary - 2, 4).unwrap(),
                [0xff, 0xfe, 0x80]
            );
            assert_eq!(
                mm.read_c_string(boundary - 2, 3),
                Err(CStringError::Unterminated)
            );
            assert_eq!(
                mm.read_c_string(usize::MAX, 0),
                Err(CStringError::Unterminated)
            );
            assert_eq!(
                mm.read_c_string(usize::MAX, 2),
                Err(CStringError::Vm(VmError::InvalidRange))
            );

            let high = arch::USER_ADDRESS_END - 2;
            assert_eq!(mm.copyout(high, &[0xfe, 0]).error, None);
            assert_eq!(mm.read_c_string(high, 32).unwrap(), [0xfe]);
            let mut high_bytes = [0; 2];
            assert_eq!(mm.copyin(&mut high_bytes, high).error, None);
            assert_eq!(high_bytes, [0xfe, 0]);

            assert_eq!(mm.copyout(usize::MAX, &[]).copied, 0);
            assert_eq!(mm.copyout(usize::MAX, &[]).error, None);
            assert_eq!(mm.copyin(&mut [], usize::MAX).error, None);
            let overflow = mm.copyout(usize::MAX - 1, &[1, 2, 3]);
            assert_eq!(
                (overflow.copied, overflow.error),
                (0, Some(VmError::InvalidRange))
            );
            let mut overflow_bytes = [0xa5; 3];
            let overflow = mm.copyin(&mut overflow_bytes, usize::MAX - 1);
            assert_eq!(
                (overflow.copied, overflow.error),
                (0, Some(VmError::InvalidRange))
            );
            assert_eq!(overflow_bytes, [0xa5; 3]);

            for address in [TARGET, boundary - 4, boundary] {
                mm.store_user_u32(address, 0x81234567).unwrap();
                assert_eq!(mm.load_user_u32(address), Ok(0x81234567));
                assert_eq!(mm.read_u32(address), Ok(0x81234567));
            }

            let child = mm.fork().unwrap();
            child.store_user_u32(TARGET + 128, 0x78563412).unwrap();
            assert_eq!(mm.load_user_u32(TARGET + 128), Ok(0));
            mm.store_user_u32(TARGET + 128, 0x12345678).unwrap();
            assert_eq!(child.load_user_u32(TARGET + 128), Ok(0x78563412));
            assert_eq!(mm.load_user_u32(TARGET + 128), Ok(0x12345678));
            drop(child);

            for address in [TARGET + 1, TARGET + 2, TARGET + 3, boundary - 2, usize::MAX] {
                assert_eq!(mm.load_user_u32(address), Err(VmError::InvalidRange));
                assert_eq!(mm.store_user_u32(address, 7), Err(VmError::InvalidRange));
            }
            assert_eq!(
                access(&user, &token, &stop, boundary, UserAccess::Read)?.1 & 0xffffffff,
                0x81234567
            );

            mm.protect(boundary, mm::PAGE_SIZE, RO).unwrap();
            let partial = mm.copyout(boundary - 2, &[11, 12, 13, 14]);
            assert_eq!(
                (partial.copied, partial.error),
                (2, Some(VmError::Permission))
            );
            let mut prefix = [0; 4];
            assert_eq!(mm.copyin(&mut prefix, boundary - 2).error, None);
            assert_eq!(prefix, [11, 12, 0x67, 0x45]);
            assert_eq!(mm.store_user_u32(boundary, 7), Err(VmError::Permission));

            mm.unmap(boundary, mm::PAGE_SIZE).unwrap();
            let mut result = [0xa5; 4];
            let partial = mm.copyin(&mut result, boundary - 2);
            assert_eq!(
                (partial.copied, partial.error),
                (2, Some(VmError::Unmapped))
            );
            assert_eq!(result, [11, 12, 0xa5, 0xa5]);
            assert_eq!(mm.read_u32(boundary - 2), Err(VmError::Unmapped));
            assert_eq!(
                mm.read_c_string(boundary - 2, 4),
                Err(CStringError::Vm(VmError::Unmapped))
            );
            assert_eq!(mm.load_user_u32(boundary), Err(VmError::Unmapped));
            let partial = mm.copyout(boundary - 2, &[21, 22, 23, 24]);
            assert_eq!(
                (partial.copied, partial.error),
                (2, Some(VmError::Unmapped))
            );
            let mut prefix = [0; 2];
            assert_eq!(mm.copyin(&mut prefix, boundary - 2).error, None);
            assert_eq!(prefix, [21, 22]);
            Ok(43)
        })?;
        fixture.joined(0, 43)
    }

    /// 原读者并持并真睡眠，写者和后来读者真实 park；最后读者放锁才依次交给写者、读者。
    #[test]
    #[ktest(min_cpus = 2)]
    fn sleeping_rwlock_order(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let controls = Controls::new(fixture)?;
        for slot in 0..2 {
            let controls = Arc::clone(&controls);
            fixture.start_task(slot, move |token, stop| {
                controls.record(slot);
                let id = task::current_task().id();
                let read = controls.lock.read();
                assert_eq!(*read, 0x9817);
                {
                    let waiter = crate::wait::Waiter::new();
                    assert!(waiter.waker().notify());
                    waiter.park();
                    assert_eq!(*read, 0x9817);
                }
                controls.ready[slot].store(true, Ordering::Release);
                controls.gates[slot].wait();
                if stop.load(Ordering::Acquire) || token.stop_requested() {
                    return Ok(0);
                }
                for target in 0..cpu::count() {
                    sched::test_migrate_to(target);
                    assert_eq!(task::current_task().id(), id);
                    assert_eq!(cpu::current().id, target);
                    assert_eq!(*read, 0x9817);
                }
                controls.phase.fetch_add(1, Ordering::AcqRel);
                drop(read);
                Ok(11 + slot as i32)
            })?;
        }
        until(
            || {
                controls.ready[..2]
                    .iter()
                    .all(|ready| ready.load(Ordering::Acquire))
                    && controls.blocked(0)
                    && controls.blocked(1)
            },
            "rwlock readers did not sleep while sharing the guard",
        )?;

        let writer = Arc::clone(&controls);
        fixture.start_task(2, move |token, stop| {
            writer.record(2);
            let mut write = writer.lock.write();
            if stop.load(Ordering::Acquire) || token.stop_requested() {
                return Ok(0);
            }
            assert_eq!(writer.phase.load(Ordering::Acquire), 2);
            assert_eq!(*write, 0x9817);
            *write = 0x9818;
            writer.phase.store(3, Ordering::Release);
            writer.ready[2].store(true, Ordering::Release);
            writer.gates[2].wait();
            if stop.load(Ordering::Acquire) || token.stop_requested() {
                return Ok(0);
            }
            sched::test_migrate_to((cpu::current().id + 1) % cpu::count());
            assert_eq!(*write, 0x9818);
            drop(write);
            Ok(13)
        })?;
        until(
            || controls.blocked(2),
            "rwlock writer did not park behind two readers",
        )?;
        assert!(controls.lock.try_read().is_none());
        assert!(controls.lock.try_write().is_none());

        let later = Arc::clone(&controls);
        fixture.start_task(3, move |token, stop| {
            later.record(3);
            let read = later.lock.read();
            if stop.load(Ordering::Acquire) || token.stop_requested() {
                return Ok(0);
            }
            assert_eq!(later.phase.load(Ordering::Acquire), 3);
            assert_eq!(*read, 0x9818);
            later.phase.store(4, Ordering::Release);
            Ok(14)
        })?;
        until(
            || controls.blocked(3),
            "late rwlock reader bypassed a waiting writer",
        )?;

        controls.gates[0].complete();
        fixture.joined(0, 11)?;
        assert_eq!(controls.phase.load(Ordering::Acquire), 1);
        assert!(controls.blocked(2));
        assert!(controls.blocked(3));

        controls.gates[1].complete();
        fixture.joined(1, 12)?;
        until(
            || controls.ready[2].load(Ordering::Acquire) && controls.blocked(2),
            "rwlock writer did not acquire after the last reader",
        )?;
        assert!(controls.blocked(3));
        assert_eq!(controls.phase.load(Ordering::Acquire), 3);

        controls.gates[2].complete();
        fixture.joined(2, 13)?;
        fixture.joined(3, 14)?;
        assert_eq!(controls.phase.load(Ordering::Acquire), 4);
        assert_eq!(*controls.lock.try_read().unwrap(), 0x9818);
        assert_eq!(*controls.lock.try_write().unwrap(), 0x9818);
        Ok(())
    }

    /// 独立顶层路径的末叶解除归还全部空表，真实用户根先离栈才进入最终全域归还账。
    #[test]
    fn last_leaf_and_inactive_root(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let address = 0x40000000;
        let (mm, mut context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        mm.map_anonymous(address, mm::PAGE_SIZE, RW, RW).unwrap();
        let mm = Arc::try_new(mm).map_err(|_| TestError("NoMemory: final leaf MM"))?;
        let controls = Controls::new(fixture)?;
        context.gpr[A5] = address;
        let thread = UserThread::try_new(Arc::clone(&mm), context)
            .map_err(|_| TestError("NoMemory: final leaf user"))?;
        let control = Arc::clone(&controls);
        fixture.start(0, thread, move |user, token, stop| {
            control.record(0);
            assert_eq!(
                access(&user, &token, &stop, address, UserAccess::Read)?,
                (1, 0)
            );
            control.gates[0].wait();
            if stop.load(Ordering::Acquire) || token.stop_requested() {
                return Ok(0);
            }
            assert_eq!(
                access(&user, &token, &stop, address, UserAccess::Write)?.0,
                0
            );
            Ok(43)
        })?;
        until(
            || controls.gates[0].test_waiting() == 1 && controls.blocked(0),
            "final leaf user did not pause after activating its root",
        )?;
        controls.gates[0].complete();
        fixture.joined(0, 43)?;

        let resident = baseline()?;
        assert!(mm.mapping(address).is_some());
        mm.unmap(address, mm::PAGE_SIZE).unwrap();
        assert!(mm.mapping(address).is_none());
        let empty = baseline()?;
        assert_eq!(
            empty.0.buddy_free_pages + empty.0.slab_backing_pages,
            resident.0.buddy_free_pages + resident.0.slab_backing_pages + arch::USER_LEVELS,
        );

        mm.map_anonymous(address, mm::PAGE_SIZE, RW, RW).unwrap();
        mm.fault(address, UserAccess::Read).unwrap();
        let refilled = baseline()?;
        assert_eq!(
            refilled.0.buddy_free_pages + refilled.0.slab_backing_pages,
            resident.0.buddy_free_pages + resident.0.slab_backing_pages,
        );

        mm.unmap(address, mm::PAGE_SIZE).unwrap();
        assert!(mm.mapping(address).is_none());
        // SLUB 的空页归还可改变临时分布，独占数据页与表页的归还总数须恢复。
        let released = baseline()?.0;
        assert_eq!(
            released.buddy_free_pages + released.slab_backing_pages,
            empty.0.buddy_free_pages + empty.0.slab_backing_pages,
        );
        drop(mm);
        Ok(())
    }

    /// 远核保持同一 Running Task 和活动根；有界握手只安排改变，结果来自后续真实用户指令。
    fn phase(
        controls: &Controls,
        expected: usize,
        token: &ChildToken,
        stop: &AtomicBool,
    ) -> TestResult {
        let end = crate::time::now()
            .checked_add(crate::time::Duration::from_secs(5))
            .ok_or(TestError("live translation deadline overflow"))?;
        while controls.phase.load(Ordering::Acquire) < expected {
            if stop.load(Ordering::Acquire) || token.stop_requested() {
                return Err(TestError("live translation participant stopped"));
            }
            if end <= crate::time::now() {
                return Err(TestError("live translation coordinator timeout"));
            }
            core::hint::spin_loop();
        }
        assert_eq!(cpu::current().id, 1);
        Ok(())
    }

    /// 不经调度安全点重进用户指令，保住上次真正命中的远核翻译以检验同步失效。
    fn live_access(
        user: &UserThread,
        token: &ChildToken,
        stop: &AtomicBool,
        direction: UserAccess,
    ) -> Result<UserTrap, TestError> {
        {
            let mut context = user.context();
            context.pc = CODE + arch::fault_offset(direction);
            context.gpr[A5] = TARGET;
        }
        let end = crate::time::now()
            .checked_add(crate::time::Duration::from_secs(5))
            .ok_or(TestError("live user deadline overflow"))?;
        loop {
            if stop.load(Ordering::Acquire) || token.stop_requested() {
                return Err(TestError("live user participant stopped"));
            }
            if end <= crate::time::now() {
                return Err(TestError("live user instruction timeout"));
            }
            assert_eq!(cpu::current().id, 1);
            let trap = user.run_user();
            if trap != UserTrap::Interrupt {
                return Ok(trap);
            }
        }
    }

    /// 撤权、COW换页、解除重建与最终解除后，同核同根重进必须观察新内容和真实拒绝方向。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_cached_translation(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        sched::test_migrate_to(0);
        let controls = Controls::new(fixture)?;
        let (mm, context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        mm.map_anonymous(TARGET, mm::PAGE_SIZE, RW, RW).unwrap();
        assert_eq!(mm.copyout(TARGET, &0x9851u64.to_le_bytes()).error, None);
        let mm = Arc::try_new(mm).map_err(|_| TestError("NoMemory: live translation MM"))?;
        let thread = UserThread::try_new(Arc::clone(&mm), context)
            .map_err(|_| TestError("NoMemory: live translation user"))?;
        let control = Arc::clone(&controls);
        fixture.start(0, thread, move |user, token, stop| {
            sched::test_migrate_to(1);
            let id = task::current_task().id();
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Read)?,
                UserTrap::Syscall
            );
            assert_eq!(user.context().gpr[T0], 0x9851);
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Write)?,
                UserTrap::Syscall
            );
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Read)?,
                UserTrap::Syscall
            );
            assert_eq!(user.context().gpr[T0], 0);

            control.ready[0].store(true, Ordering::Release);
            phase(&control, 1, &token, &stop)?;
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Write)?,
                UserTrap::PageFault {
                    address: TARGET,
                    access: UserAccess::Write,
                }
            );
            assert_eq!(
                user.mm().unwrap().fault(TARGET, UserAccess::Write),
                Err(VmError::Permission)
            );
            control.ready[1].store(true, Ordering::Release);

            phase(&control, 2, &token, &stop)?;
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Read)?,
                UserTrap::Syscall
            );
            assert_eq!(user.context().gpr[T0], 0x9852);
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Write)?,
                UserTrap::Syscall
            );
            control.ready[2].store(true, Ordering::Release);

            phase(&control, 3, &token, &stop)?;
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Read)?,
                UserTrap::Syscall
            );
            assert_eq!(user.context().gpr[T0], 0x9853);
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Write)?,
                UserTrap::PageFault {
                    address: TARGET,
                    access: UserAccess::Write,
                }
            );
            control.ready[3].store(true, Ordering::Release);

            phase(&control, 4, &token, &stop)?;
            assert_eq!(
                live_access(&user, &token, &stop, UserAccess::Read)?,
                UserTrap::PageFault {
                    address: TARGET,
                    access: UserAccess::Read,
                }
            );
            assert_eq!(
                user.mm().unwrap().fault(TARGET, UserAccess::Read),
                Err(VmError::Unmapped)
            );
            assert_eq!(task::current_task().id(), id);
            Ok(43)
        })?;
        until(
            || controls.ready[0].load(Ordering::Acquire),
            "remote user did not warm its translation",
        )?;

        sched::test_migrate_to(0);
        mm.protect(TARGET, mm::PAGE_SIZE, RO).unwrap();
        controls.phase.store(1, Ordering::Release);
        until(
            || controls.ready[1].load(Ordering::Acquire),
            "remote user retained stale write permission",
        )?;

        sched::test_migrate_to(0);
        mm.protect(TARGET, mm::PAGE_SIZE, RW).unwrap();
        let old = mm.fork().unwrap();
        let copied = mm.copyout(TARGET, &0x9852u64.to_le_bytes());
        assert_eq!((copied.copied, copied.error), (8, None));
        assert_eq!(word(&old, TARGET), 0);
        assert_ne!(
            old.mapping(TARGET).unwrap().physical,
            mm.mapping(TARGET).unwrap().physical
        );
        controls.phase.store(2, Ordering::Release);
        until(
            || controls.ready[2].load(Ordering::Acquire),
            "remote user retained stale page contents",
        )?;

        sched::test_migrate_to(0);
        let replaced = mm.fork().unwrap();
        mm.unmap(TARGET, mm::PAGE_SIZE).unwrap();
        mm.map_anonymous(TARGET, mm::PAGE_SIZE, RW, RW).unwrap();
        assert_eq!(mm.copyout(TARGET, &0x9853u64.to_le_bytes()).error, None);
        mm.protect(TARGET, mm::PAGE_SIZE, RO).unwrap();
        assert_ne!(
            replaced.mapping(TARGET).unwrap().physical,
            mm.mapping(TARGET).unwrap().physical
        );
        assert_eq!(word(&replaced, TARGET), 0);
        controls.phase.store(3, Ordering::Release);
        until(
            || controls.ready[3].load(Ordering::Acquire),
            "remote user did not observe rebuilt read-only leaf",
        )?;

        sched::test_migrate_to(0);
        mm.unmap(TARGET, mm::PAGE_SIZE).unwrap();
        controls.phase.store(4, Ordering::Release);
        fixture.joined(0, 43)?;

        drop(replaced);
        drop(old);
        drop(mm);
        Ok(())
    }
}
