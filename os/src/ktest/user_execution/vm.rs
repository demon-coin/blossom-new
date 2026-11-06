/// 用户虚拟内存管理（`Mm`）、按需分页、写时复制（COW）、安全访存（`uaccess`）与跨核 TLB 同步测试套件。
///
/// # 模块职责与架构定位
/// 本模块作为 `user_execution` 下的子测试套件（标签 `user-vm`），结合真实用户态指令陷入与内核态页表操作，
/// 端到端验证用户地址空间的完整生命周期与并发正确性。
/// - **上游调度者**：`ktest` 框架通过 `#[ktest::suite]` 宏注册并驱动本套件中的各个测试用例。
/// - **下游被测模块**：`mm::space::Mm`（匿名映射、缺页处理、fork 写时复制、权限保护、解除映射）、
///   `mm::uaccess`（跨页拷贝、C 字符串读取、对齐原子读写）、`sync::RwLock`（可睡眠读写锁公平性）以及跨核 TLB 一致性维护。
///
/// # 核心测试场景与状态流转
///
/// ```text
/// [匿名映射 map_anonymous]
///        |
///        | 首次用户 load/store 或 copyout 触发缺页 (mm.fault)
///        v
/// [独占可写叶子页 (RW)]
///        |
///        | mm.fork() 共享物理页并降级双方 PTE 写权限为只读
///        v
/// [父子共享只读 COW 页 (RO, 允许未来写)]
///        |
///        +---> 单方 copyout / 用户 store / 并发写缺页 ---> 分配新物理页并复制内容，恢复独占 RW
///        |
///        +---> mm.unmap() 解除最后一个叶子映射 ----------> 级联回收各级空页表页至页分配器
/// ```
///
/// # 并发与同步契约
/// - 并发写时复制测试（`readonly_future_write_and_racing_faults`）使用 `Controls` 屏障让两个 CPU 上的用户线程同时触发写缺页，验证 `Mm` 内部锁对叶子页表的二次检查不会丢失更新或产生重复拆分。
/// - 跨核 TLB 同步测试（`remote_cached_translation`）让远端 CPU 1 持续运行同一用户线程且不经过调度切换，由 CPU 0 修改页表权限、触发 COW 或解除映射，验证跨核 TLB 击落（Shootdown）的同步生效。
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

    /// 验证匿名映射在首次用户指令访存时才按需分配并清零物理页，且跨页 `copyin` 与 `unmap` 后访问行为符合预期。
    ///
    /// - **测试目标**：确认 `map_anonymous` 仅建立虚拟区间记录而不预先分配叶子页；首次读/写缺页分配零页后，后续读写不再缺页，解除映射后再次访问返回 `VmError::Unmapped`。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 映射 3 个连续匿名页并断言初始无物理页映射；
    ///   2. 依次对前两页触发用户态读缺页、`copyout` 写入与用户态写清零，对第三页直接触发用户态写缺页；
    ///   3. 执行跨页边界的 `copyin` 检验零页内容，最后 `unmap` 全部 3 页并验证再次用户态读取产生无法修复的 `Unmapped` 缺页。
    #[test]
    fn anonymous_first_touch(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (mm, context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        mm.map_anonymous(TARGET, 3 * mm::PAGE_SIZE, RW, RW).unwrap();
        assert!(mm.mapping(TARGET).is_none());
        assert!(mm.mapping(TARGET + mm::PAGE_SIZE).is_none());
        assert!(mm.mapping(TARGET + 2 * mm::PAGE_SIZE).is_none());
        fixture.start(0, user(mm, context)?, move |user, token, stop| {
            let mm = user.mm().unwrap();
            // Step 1: 验证前两个匿名页首次读取触发 1 次缺页并读出 0，写入后再由用户态写指令清零不再触发缺页。
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

            // Step 2: 验证第三个匿名页首次直接由用户态写指令触发 1 次按需分配缺页，随后跨页 copyin 读出全零。
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

            // Step 3: 解除全部 3 个匿名页映射，验证用户态再次读取触发缺页且 mm.fault 返回 Unmapped。
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

    /// 验证 `fork` 后的写时复制（COW）语义：内核 `copyout` 与用户态 `store` 各自独立触发拆页，父子地址空间互不污染且未写入页保持物理页共享。
    ///
    /// - **测试目标**：确认 `fork` 将父子页表项均置为只读并共享同一物理页；任何一方通过 `copyout` 或用户态写指令修改页内容时均会分配独立物理页，而未修改的相邻页继续共享。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 在父地址空间映射 2 个匿名页并写入 `0x9851`，调用 `fork()` 创建子地址空间，断言双方共享物理页且写权限均被清除；
    ///   2. 对子地址空间第 1 页执行 `copyout` 写入 `0x9852`，断言第 1 页物理地址已分离、第 2 页仍共享；
    ///   3. 启动绑定父地址空间的用户线程对第 1 页执行写指令（写入 0），断言父地址空间第 1 页变为 0，而子地址空间两页数据保持不变。
    #[test]
    fn fork_copyout_and_user_store(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (parent, context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        // Step 1: 在父地址空间映射两个匿名页并写入初始值，随后 fork 出子地址空间并验证 COW 初始只读共享状态。
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

        // Step 2: 子地址空间通过 copyout 修改第一页，验证第一页完成 COW 拆页且第二页继续共享。
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

        // Step 3: 父地址空间通过用户态 store 指令修改第一页，验证父子内容彻底隔离。
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

    /// 验证只读但最大权限允许写入的页在 `fork` 及权限升降级后仍保持写时复制（COW）语义，并在双核并发写缺页时正确串行化。
    ///
    /// - **测试目标**：确认当前权限为 `RO` 但最大权限为 `RW` 的页在 `fork` 后恢复 `RW` 权限时不会直接授予写权限，且两个 CPU 分别代表父子 `Mm`（或共享同一 `Mm`）同时处理写缺页时不会破坏物理页隔离。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 分别针对父子不同 `Mm`（`same_mm = false`）与多线程共享同一 `Mm`（`same_mm = true`）两种模式构造共享只读页；
    ///   2. 将页面权限在 `RW` 与 `RO` 间多次切换后恢复为 `RW`，在两个 CPU 上启动用户线程同时执行写指令触发 `PageFault`；
    ///   3. 使用同步屏障让两个线程同时调用 `mm.fault(TARGET, Write)`，验证修复后双方写指令成功提交且物理页按预期拆分或共享。
    #[test]
    #[ktest(min_cpus = 2)]
    fn readonly_future_write_and_racing_faults(
        _: &mut TestContext<'_>,
        fixture: &mut Users,
    ) -> TestResult {
        for same_mm in [false, true] {
            // Step 1: 准备初始写入 0x9851 后降权为只读的页面，执行 fork 并反复升降权限至 RW。
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

            // Step 2: 在两个 CPU 上启动用户线程，先同时触发写缺页并停在同步屏障前，再并发执行 mm.fault。
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

            // Step 3: 等待双核用户线程均进入写缺页后同时放行，并校验最终页内容与物理页拆分结果。
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

    /// 验证用户空间安全访存接口（`copyin`、`copyout`、`read_c_string`、`load_user_u32`、`store_user_u32`）在跨页边界、地址溢出、非对齐及权限/未映射故障下的精确边界行为。
    ///
    /// - **测试目标**：确认跨页拷贝支持任意偏移与长度；当后半页只读或未映射时，前半页已成功传输的字节数（Partial Progress）被准确报告；对齐原子读写拒绝非对齐地址且遵守 COW 语义。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 映射两个相邻页与用户空间最高页，遍历跨页边界的各种偏移和长度验证 `copyout`/`copyin` 完整性；
    ///   2. 验证跨页整数读取、`read_c_string` 终止符截断与 `usize::MAX` 溢出拦截；
    ///   3. 验证 `load_user_u32`/`store_user_u32` 的 `fork` COW 隔离与非对齐拒绝；
    ///   4. 将第二页先设为只读再解除映射，验证跨页 `copyout`/`copyin` 返回已成功传输的前缀字节数（`copied = 2`）及对应错误码。
    #[test]
    fn bounded_uaccess(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let (mm, context, _) = image(arch::fault_offset(UserAccess::Read), 0)?;
        mm.map_anonymous(TARGET, 2 * mm::PAGE_SIZE, RW, RW).unwrap();
        let high = arch::USER_ADDRESS_END - mm::PAGE_SIZE;
        mm.map_anonymous(high, mm::PAGE_SIZE, RW, RW).unwrap();
        fixture.start(0, user(mm, context)?, move |user, token, stop| {
            let mm = user.mm().unwrap();
            // Step 1: 遍历页边界附近的偏移与长度，验证跨页 copyout 与 copyin 不会越界或污染邻接字节。
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

            // Step 2: 验证跨页基本整数读取、C 字符串读取以及地址空间末尾与算术溢出边界。
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

            // Step 3: 验证 32 位对齐用户态读写接口的正常读写、fork 写时复制隔离以及非对齐地址拒绝。
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

            // Step 4: 将后一页改为只读再解除映射，验证跨页访问在遇到权限或未映射错误时准确返回前半页已完成的字节数。
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

    /// 验证可睡眠读写锁（`RwLock`）的读者共享睡眠、写者阻塞排队以及写者优先防饥饿唤醒顺序。
    ///
    /// - **测试目标**：确认多个读者可同时持锁睡眠并跨核迁移；当有写者排队等待时，新到达的读者必须排在写者之后阻塞，直到最后一个旧读者释放锁后先交由写者独占修改，写者释放后再唤醒后续读者。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 启动槽位 0、1 两个读者任务获取读锁并进入睡眠，确认两者共享读锁；
    ///   2. 启动槽位 2 写者任务请求写锁并确认其阻塞，随后启动槽位 3 新读者任务并确认其因写者等待而同样阻塞；
    ///   3. 依次放行槽位 0、1 两个旧读者，确认仅当第二个读者释放锁后写者才获得锁并将值从 `0x9817` 改为 `0x9818`；
    ///   4. 放行写者释放写锁，确认槽位 3 后续读者最后获得锁并读到写者更新后的 `0x9818`。
    #[test]
    #[ktest(min_cpus = 2)]
    fn sleeping_rwlock_order(_: &mut TestContext<'_>, fixture: &mut Users) -> TestResult {
        let controls = Controls::new(fixture)?;
        // Step 1: 启动两个初始读者共同持有读锁并阻塞在各自门控上。
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

        // Step 2: 启动写者与后续读者，验证写者在两个活跃读者之后睡眠等待，且后续读者不会越过等待中的写者。
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

        // Step 3: 依次放行两个初始读者和写者，校验“全部初始读者 -> 写者 -> 后续读者”的严格交接顺序。
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

    /// 验证解除独立顶层子树下的最后一个叶子映射时会级联释放全部空页表页，并在非活动用户地址空间重新映射和销毁时无物理页泄漏。
    ///
    /// - **测试目标**：确认当用户线程退出且 `Mm` 不再处于任何 CPU 的活动页表状态后，`unmap` 唯一叶子页会将叶子页与各级中间页表页（共 `arch::USER_LEVELS` 页）全部归还给内存分配器。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. 在独立高地址 `0x40000000` 映射 1 页并由用户线程触发缺页建表，等待用户线程彻底退出；
    ///   2. 采集当前空闲页基线 `resident`，调用 `unmap` 解除该页映射，验证系统空闲物理页（伙伴系统空闲页 + SLUB 后备页）精确增加 `arch::USER_LEVELS` 页；
    ///   3. 重新映射并触发缺页后再解除映射，验证页表页的二次分配与回收账目完全平衡。
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
        // Step 1: 让用户线程触发缺页激活整条页表路径，随后等待用户线程退出。
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

        // Step 2: 解除该子树下的唯一叶子映射，验证数据页与各级空页表页全部归还。
        let resident = baseline()?;
        assert!(mm.mapping(address).is_some());
        mm.unmap(address, mm::PAGE_SIZE).unwrap();
        assert!(mm.mapping(address).is_none());
        let empty = baseline()?;
        assert_eq!(
            empty.0.buddy_free_pages + empty.0.slab_backing_pages,
            resident.0.buddy_free_pages + resident.0.slab_backing_pages + arch::USER_LEVELS,
        );

        // Step 3: 重新建立映射并再次解除，验证页表页二次分配与回收后的总页数守恒。
        mm.map_anonymous(address, mm::PAGE_SIZE, RW, RW).unwrap();
        mm.fault(address, UserAccess::Read).unwrap();
        let refilled = baseline()?;
        assert_eq!(
            refilled.0.buddy_free_pages + refilled.0.slab_backing_pages,
            resident.0.buddy_free_pages + resident.0.slab_backing_pages,
        );

        mm.unmap(address, mm::PAGE_SIZE).unwrap();
        assert!(mm.mapping(address).is_none());
        // SLUB 空闲 slab 页向伙伴系统的回收可能改变两者之间的分布，但伙伴系统空闲页与 SLUB 后备页之和必须严格恢复。
        let released = baseline()?.0;
        assert_eq!(
            released.buddy_free_pages + released.slab_backing_pages,
            empty.0.buddy_free_pages + empty.0.slab_backing_pages,
        );
        drop(mm);
        Ok(())
    }

    /// 在远端 CPU 1 上自旋等待协调者将阶段号推进到 `expected`，期间不让出 CPU，以保持当前活动页表根与 TLB 项驻留。
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

    /// 在远端 CPU 1 上不经过调度器切换直接进入用户态执行访存指令，用于检验本核缓存的旧 TLB 翻译是否已被同步击落。
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

    /// 验证当一个 CPU 上的用户线程持续运行且缓存了某虚拟页的 TLB 翻译时，另一 CPU 对该 `Mm` 执行权限收紧、写时复制拆页、重建只读映射及解除映射后，远端 CPU 均能同步观察到最新页表状态。
    ///
    /// - **测试目标**：确认 `protect`、`fork` + `copyout`（COW 换页）、`unmap` + 重新映射等页表修改操作会正确触发跨核 TLB 同步失效，远端正在运行的用户线程不会继续命中过期的读写权限或旧物理页。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. CPU 1 上的用户线程对 `TARGET` 执行读写访问以预热本地 TLB（将值清零），随后通知 CPU 0 进入阶段 1；
    ///   2. 阶段 1：CPU 0 将 `TARGET` 权限收紧为 `RO`，验证 CPU 1 用户线程再次写入立即触发 `VmError::Permission` 缺页；
    ///   3. 阶段 2：CPU 0 恢复 `RW`、执行 `fork` 并通过 `copyout` 写入 `0x9852` 触发 COW 换页，验证 CPU 1 用户线程读取到新页内容 `0x9852`；
    ///   4. 阶段 3：CPU 0 再次 `fork`、`unmap` 后重建映射写入 `0x9853` 并设为 `RO`，验证 CPU 1 读到 `0x9853` 且写入触发缺页；
    ///   5. 阶段 4：CPU 0 彻底 `unmap` 该页，验证 CPU 1 用户线程读取触发 `VmError::Unmapped` 缺页。
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
        // Step 1: 在 CPU 1 上启动用户线程预热 TLB 并按阶段验证每次跨核页表更新后的访存行为。
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

        // Step 2: 阶段 1 —— CPU 0 将页面降权为只读，验证 CPU 1 写访问被拒绝。
        sched::test_migrate_to(0);
        mm.protect(TARGET, mm::PAGE_SIZE, RO).unwrap();
        controls.phase.store(1, Ordering::Release);
        until(
            || controls.ready[1].load(Ordering::Acquire),
            "remote user retained stale write permission",
        )?;

        // Step 3: 阶段 2 —— CPU 0 恢复可写并通过 fork + copyout 触发 COW 换页，验证 CPU 1 观察到新物理页数据。
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

        // Step 4: 阶段 3 —— CPU 0 解除映射后重建只读叶子页，验证 CPU 1 观察到新只读数据且写操作触发缺页。
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

        // Step 5: 阶段 4 —— CPU 0 最终解除映射，验证 CPU 1 读操作触发 Unmapped 缺页并等待线程退出。
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
