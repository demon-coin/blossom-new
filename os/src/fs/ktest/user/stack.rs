//! 参数预算和主栈增长使用真实用户 ELF，离线 Mm 只消费事先登记的栈范围。

use super::*;
use crate::arch::UserAccess;
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;

const PROGRAM: &[u8] = b"/test/selftests/process/exec_stack";
const STACK_BASE: usize = 0x3500_0000;

const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-exec-stack"), resources("root"), timeout_ms = 180000)]
pub(crate) mod behavior {
    use super::*;

    /// 真正 exec 后由新 CRT 检查参数和环境，完整消费2MiB、单串、空向量及坏指针边界。
    #[test]
    #[ktest(tags("user-exec-arguments"))]
    fn arguments_and_environment_use_linux_budget(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--arguments"], Credentials::ROOT)
    }

    /// 当前用户运行中修改软栈限额，实际缺页、uaccess、guard、切片和 fork 都经过正式 Mm。
    #[test]
    #[ktest(tags("user-stack-growth"))]
    fn current_limits_control_real_stack_growth(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--stack"], Credentials::ROOT)
    }

    /// 内核 worker 持有的离线栈只允许已登记范围缺页，不能借用无关用户限额扩大缺口。
    #[test]
    #[ktest(tags("user-stack-offline"))]
    fn offline_mm_does_not_borrow_current_stack_limit(
        context: &mut TestContext<'_>,
        _: &mut UserOwner,
    ) -> TestResult {
        let mm = Mm::try_new().map_err(|_| TestError("offline stack Mm preparation failed"))?;
        mm.map_stack(STACK_BASE, 2 * PAGE_SIZE, RW, RW)
            .map_err(|_| TestError("offline initial stack registration failed"))?;

        let initial = mm.copyout(STACK_BASE, &[0x35]);
        context.expect_eq((initial.copied, initial.error), (1, None));
        context.expect_eq(mm.fault(STACK_BASE + PAGE_SIZE, UserAccess::Write), Ok(()));
        context.expect_eq(
            mm.fault(STACK_BASE - PAGE_SIZE, UserAccess::Write),
            Err(VmError::Unmapped),
        );
        let outside = mm.copyout(STACK_BASE - PAGE_SIZE, &[0x53]);
        context.expect_eq(
            (outside.copied, outside.error),
            (0, Some(VmError::Unmapped)),
        );
        Ok(())
    }
}
