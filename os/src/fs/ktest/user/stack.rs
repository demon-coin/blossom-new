//! `execve` 参数与环境变量预算、用户主栈按需增长限额（`RLIMIT_STACK`）及离线 `Mm` 栈边界隔离测试套件。
//!
//! # 模块职责与架构定位
//! 本模块定义 `behavior` 内核测试套件（`#[ktest::suite]`），负责验证：
//! 1. `execve` 在构建新用户栈时的参数（`argv`）与环境变量（`envp`）总字节预算（2 MiB）、单字符串上限与非法指针检查；
//! 2. 运行中进程动态调整 `RLIMIT_STACK` 软限额后，页错误处理（Page Fault）、内核 `uaccess` 拷贝、栈保护页（Guard Page）及 `fork` 地址空间复制的栈增长边界；
//! 3. 内核线程直接创建的独立离线 `Mm` 地址空间仅在其显式注册的栈区间内响应按需缺页，不会错误借用当前运行任务的 `RLIMIT_STACK` 向下扩栈。
//!
//! # 核心设计与关键布局
//! ## 离线 `Mm` 栈映射与越界缺页测试布局（`offline_mm_does_not_borrow_current_stack_limit`）
//! ```text
//! 虚拟地址方向 (低 -> 高)
//! +------------------------+------------------------+------------------------+
//! | STACK_BASE - PAGE_SIZE |       STACK_BASE       | STACK_BASE + PAGE_SIZE |
//! |      0x34ff_f000       |      0x3500_0000       |      0x3500_1000       |
//! +------------------------+------------------------+------------------------+
//! | 未注册区域 (Unmapped)   | <----- 已注册 2 页离线栈范围 (2 * PAGE_SIZE) -----> |
//! | fault / copyout 均拒绝  | copyout 正常写入 1 字节 | fault(Write) 正常分配页 |
//! +------------------------+------------------------+------------------------+
//! ```
//!
//! # 锁获取顺序与并发契约
//! - 本模块不直接获取自旋锁；离线地址空间测试在独立新建的 `Mm` 实例上同步调用 `map_stack`、`copyout` 与 `fault`，无跨线程共享。
//! - 用户态自测程序通过 `UserOwner::run_selftest_named` 启动，在完整回收子进程树与输出收集线程后返回。

use super::*;
use crate::arch::UserAccess;
use crate::mm::user_table::PagePermissions;
use crate::mm::PAGE_SIZE;

/// 用户态 `exec` 参数预算与主栈增长自测程序路径。
const PROGRAM: &[u8] = b"/test/selftests/process/exec_stack";
/// 离线地址空间栈边界测试使用的栈底起始虚拟地址（字节单位，页对齐）。
const STACK_BASE: usize = 0x3500_0000;

/// 可读可写、不可执行的用户页权限常量。
const RW: PagePermissions = PagePermissions {
    read: true,
    write: true,
    execute: false,
};

#[ktest::suite(fixture = UserOwner, suite_resources = UserSuite, context = "Thread", tags("user", "user-exec-stack"), resources("root"), timeout_ms = 180000)]
pub(crate) mod behavior {
    use super::*;

    /// 测试目标：验证 `execve` 加载新程序时对命令行参数（`argv`）与环境变量（`envp`）的 2 MiB 预算控制、单字符串长度上限、空向量处理以及坏指针错误返回（`EFAULT` / `E2BIG`）。
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 准备只读根文件系统与 `/dev/console` 标准流环境；
    /// 2. When: 以 `ROOT` 凭据运行 `/test/selftests/process/exec_stack --arguments` 自测程序；
    /// 3. Then: 新程序 CRT 启动后核对参数与环境布局完整无误，各项边界检查全部通过并以状态码 0 退出。
    #[test]
    #[ktest(tags("user-exec-arguments"))]
    fn arguments_and_environment_use_linux_budget(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--arguments"], Credentials::ROOT)
    }

    /// 测试目标：验证用户进程在运行期修改 `RLIMIT_STACK` 软限额后，指令缺页、内核 `uaccess`、栈底保护页（Guard Page）以及 `fork` 子进程均严格遵循当前主栈限额。
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 准备只读根文件系统与 `/dev/console` 环境；
    /// 2. When: 以 `ROOT` 凭据运行 `/test/selftests/process/exec_stack --stack` 自测程序；
    /// 3. Then: 限额内的栈向下扩展按需分配物理页成功，超出当前软限额或触碰保护页的访问被正确拦截，自测程序以状态码 0 退出。
    #[test]
    #[ktest(tags("user-stack-growth"))]
    fn current_limits_control_real_stack_growth(
        context: &mut TestContext<'_>,
        fixture: &mut UserOwner,
    ) -> TestResult {
        fixture.prepare_tty(context, 4)?;
        fixture.run_selftest_named(context, PROGRAM, &[b"--stack"], Credentials::ROOT)
    }

    /// 测试目标：验证未绑定用户线程的离线 `Mm` 地址空间仅允许在事先通过 `map_stack` 登记的固定栈区间内触发缺页分配，不会借用当前任务的栈限额向下扩展未登记页。
    /// 场景步骤（Given-When-Then）：
    /// 1. Given: 创建独立的离线 `Mm` 实例，并在 `[STACK_BASE, STACK_BASE + 2 * PAGE_SIZE)` 登记 2 页可读写栈空间；
    /// 2. When: 对已登记区间执行 `copyout` 与写 `fault`，并对低于 `STACK_BASE` 一页的地址（`STACK_BASE - PAGE_SIZE`）执行写 `fault` 与 `copyout`；
    /// 3. Then: 已登记区间内的写入与缺页全部成功，而 `STACK_BASE - PAGE_SIZE` 处的缺页与 `copyout` 均返回 `VmError::Unmapped`。
    #[test]
    #[ktest(tags("user-stack-offline"))]
    fn offline_mm_does_not_borrow_current_stack_limit(
        context: &mut TestContext<'_>,
        _: &mut UserOwner,
    ) -> TestResult {
        let mm = Mm::try_new().map_err(|_| TestError("offline stack Mm preparation failed"))?;
        mm.map_stack(STACK_BASE, 2 * PAGE_SIZE, RW, RW)
            .map_err(|_| TestError("offline initial stack registration failed"))?;

        // Step 1: 验证在已登记的 2 页栈区间（STACK_BASE 与 STACK_BASE + PAGE_SIZE）内，copyout 与按需写缺页均能成功。
        let initial = mm.copyout(STACK_BASE, &[0x35]);
        context.expect_eq((initial.copied, initial.error), (1, None));
        context.expect_eq(mm.fault(STACK_BASE + PAGE_SIZE, UserAccess::Write), Ok(()));
        // Step 2: 验证访问栈底以下的未登记页（STACK_BASE - PAGE_SIZE）不会触发自动向下扩栈，fault 与 copyout 均返回 Unmapped。
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
