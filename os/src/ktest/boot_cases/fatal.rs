//! 启动阶段（Boot）致命异常与从核（AP）上线超时隔离测试套件。
//!
//! # 模块定位
//! 本模块隶属于内核自检框架（`ktest`）的启动阶段用例集（`boot_cases`），用于在每个用例独占的
//! 虚拟机实例（`isolation = "Case"`）中验证不可恢复的硬件异常（Trap）和从核（AP）启动超时路径。
//! 上游由 `ktest` 启动阶段分发器直接调用，下游调用架构异常触发桩（`arch::trigger_*`）、内存管理初始化
//! （`mm::initialize`）以及多核启动接口（`cpu::prepare_secondary_stacks`、`cpu::start_secondaries`）。
//!
//! # 核心机制与边界切换布局
//! 普通 `ktest` 用例运行在可恢复的 Panic 捕获边界（`Boundary`）之内；而本模块中的致命用例需要检验
//! 异常控制流直接进入底层硬件 Trap 处理程序或触发不可捕获的停机 Panic，因此通过 `UncaughtBoot` 守卫
//! 临时卸载当前 CPU 的恢复边界：
//!
//! ```text
//! ktest 框架外层分发
//!   │  (Pin 住外层 Boundary，注册到当前 CPU 的 Boot 边界槽)
//!   ▼
//! UncaughtBoot::enter()
//!   ├─ 1. irq_save() 关闭中断，防止切换期间发生重入
//!   ├─ 2. replace_boundary(null_mut()) 清空恢复槽，保存原 Boundary 指针
//!   └─ 3. enable_interrupts() 开启中断，执行致命测试逻辑
//!          │
//!          ├─► [正常触发致命异常/超时 Panic] ──► 发现恢复槽为空 ──► 输出致命报告并终止 VM
//!          │
//!          └─► [异常未触发，意外返回] ──► UncaughtBoot::drop()
//!                                           ├─ 1. irq_save() 关闭中断
//!                                           ├─ 2. replace_boundary(previous) 恢复外层 Boundary
//!                                           └─ 3. irq_restore(irq) 恢复中断状态，由外层 panic! 捕获报错
//! ```
//!
//! # 锁与并发契约
//! - 本模块所有用例均在 BSP（逻辑核 0）单核启动阶段执行，执行前多核调度器尚未启动。
//! - 修改全局/本核异常恢复边界槽（`replace_boundary`）或初始化伙伴分配器与动态从核栈时，
//!   必须先通过 `arch::irq_save()` 或 `IrqSaveGuard` 关闭本地中断，严禁在持锁或切换边界期间响应中断。

use crate::ktest::panic::Boundary;
use crate::{arch, boot, cpu, mm};
use ::ktest::TestContext;

/// 临时卸载启动阶段 Panic 恢复边界的 RAII 守卫。
///
/// 在作用域内将当前 CPU 的 `ktest` 恢复边界槽置空，使随后的 Panic 或硬件 Trap 直接走内核致命错误路径；
/// 若被测代码意外未触发终止而正常退出作用域，则在析构时还原外层恢复边界与中断状态。
struct UncaughtBoot {
    /// 进入守卫前外层框架注册的 `Boundary` 指针（生命周期由外层调用栈 `Pin` 保证）。
    previous: *mut Boundary,
    /// 进入守卫前保存的本地中断状态字（架构相关的 CSR 状态位）。
    irq: usize,
}
impl UncaughtBoot {
    /// 卸载当前启动阶段的 Panic 可恢复捕获边界，并开启本地中断。
    ///
    /// # 调用上下文
    /// 仅限在 BSP（CPU 0）启动阶段、`ktest` 用例上下文中调用。
    fn enter() -> Self {
        // Step 1: 保存并关闭本地中断，保证边界指针替换过程的原子性。
        let irq = arch::irq_save();
        // SAFETY:
        // 1. 当前运行在 BSP 独占启动栈上，外层分发函数持有的 `Boundary` 已被 `Pin` 在栈上，生命周期覆盖整个用例；
        // 2. 本地中断已通过上一行 `arch::irq_save()` 关闭，不会发生中断重入访问恢复边界槽；
        // 3. 仅将当前 Boot 边界槽替换为空指针并保存原指针，若后续未停机则由 `Drop` 负责原样恢复。
        let previous = unsafe { crate::ktest::replace_boundary(core::ptr::null_mut()) };
        // Step 2: 重新开启本地中断，允许被测路径依赖时钟中断或 IPI。
        arch::enable_interrupts();
        Self { previous, irq }
    }
}
impl Drop for UncaughtBoot {
    /// 当致命错误未按预期发生而退回正常控制流时，先关闭中断恢复原恢复边界，再还原进入前的中断状态。
    fn drop(&mut self) {
        arch::irq_save();
        // SAFETY:
        // 1. `self.previous` 来自 `enter()` 从外层栈帧捕获的 `Pin` 边界指针，在当前函数返回外层分发器前始终有效；
        // 2. 当前仍在 BSP 上执行且上一行已关闭本地中断，恢复边界槽与中断状态期间不存在并发竞争。
        unsafe {
            crate::ktest::replace_boundary(self.previous);
            arch::irq_restore(self.irq);
        }
    }
}

#[cfg(target_arch = "riscv64")]
#[::ktest::suite(
    context = "Boot",
    isolation = "Case",
    tags("boot", "fatal", "trap"),
    timeout_ms = 1000
)]
pub(crate) mod traps {
    use super::*;
    /// 测试目标：验证 RISC-V 64 架构下执行非法指令时能触发硬件异常并走致命 Trap 报告路径。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 独占启动阶段 VM 环境。
    /// - When: 调用 `arch::trigger_fault()` 执行一条非法指令。
    /// - Then: CPU 陷入 S-mode Trap 处理程序，报告 `expected_fatal = "illegal"`（宿主机校验 PC/SP 非零且 `scause == 2`）。
    #[test]
    #[ktest(expected_fatal = "illegal")]
    fn illegal_instruction() {
        arch::trigger_fault();
    }
}

#[cfg(target_arch = "loongarch64")]
#[::ktest::suite(
    context = "Boot",
    isolation = "Case",
    tags("boot", "fatal", "trap"),
    timeout_ms = 1000
)]
pub(crate) mod traps {
    use super::*;
    /// 测试目标：验证 LoongArch64 架构下执行非法指令时能触发硬件异常并走致命 Trap 报告路径。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 独占启动阶段 VM 环境。
    /// - When: 调用 `arch::trigger_fault()` 执行一条非法指令。
    /// - Then: CPU 陷入异常入口，报告 `expected_fatal = "illegal"`（宿主机校验 PC/SP 非零且 `ESTAT.Ecode == 13`）。
    #[test]
    #[ktest(expected_fatal = "illegal")]
    fn illegal_instruction() {
        arch::trigger_fault();
    }

    /// 测试目标：验证 LoongArch64 架构下访问直接映射窗口（DMW）之外的未映射低地址 `0x1000` 时，
    /// 能够进入专用的 TLB 重填（TLB Refill）异常入口。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 独占启动阶段 VM 环境，地址 `0x1000` 不在任何 DMW 窗口或有效页表映射中。
    /// - When: 调用 `arch::trigger_unmapped_fault()` 对 `0x1000` 发起读取。
    /// - Then: 触发 TLB 重填异常并报告 `expected_fatal = "refill"`（宿主机校验 `IsTLBR == 1` 且 `TLBRBADV == 0x1000`）。
    #[test]
    #[ktest(expected_fatal = "refill")]
    fn unmapped_refill() {
        arch::trigger_unmapped_fault();
    }
}

#[::ktest::suite(
    context = "Boot",
    isolation = "Case",
    tags("boot", "fatal", "ap-startup"),
    resources("ap-freeze"),
    min_cpus = 2,
    timeout_ms = 10000
)]
pub(crate) mod startup {
    use super::*;
    /// 测试目标：验证当从核（AP）被宿主机冻结无法上线时，主核（BSP）在完成内存初始化与从核栈分配后，
    /// 调用 `cpu::start_secondaries()` 会在等待 5 秒后触发预期的超时 Panic。
    ///
    /// 场景步骤（Given-When-Then）：
    /// - Given: 至少 2 个 CPU 的独占 VM 环境，宿主机通过 `ap-freeze` 资源将从核冻结在启动入口前；
    ///   当前在 BSP（CPU 0）上通过 `UncaughtBoot::enter()` 禁用可恢复 Panic 捕获。
    /// - When:
    ///   1. 在关中断保护下调用 `mm::initialize` 初始化物理内存管理（伙伴系统、PCP、SLUB）与最终内核页表；
    ///   2. 调用 `cpu::prepare_secondary_stacks` 为所有从核分配动态启动栈；
    ///   3. 校验所有从核处于离线状态并输出诊断信息；
    ///   4. 调用 `cpu::start_secondaries` 唤醒从核并等待其上线。
    /// - Then: 因从核被冻结无法置位 `online` 标志，主核等待超时后触发不可捕获的 `"AP online timed out"` Panic 并终止 VM。
    #[test]
    #[ktest(expected_fatal = "panic", fatal_message = "AP online timed out")]
    fn frozen_ap(context: &mut TestContext<'_>) {
        assert_eq!(cpu::current().id, 0);
        // Step 1: 卸载外层可恢复 Panic 边界，确保后续超时 Panic 或意外上线 Panic 直接终止当前独占 VM。
        let _uncaught = UncaughtBoot::enter();
        {
            // Step 2: 关闭本地中断，单核独占完成物理内存子系统初始化与从核动态启动栈分配。
            let _irq = crate::sync::IrqSaveGuard::new();
            // SAFETY:
            // 1. 当前用例运行在 `isolation = "Case"` 的全新独占 VM 中，此前尚未调用过 `mm::initialize`；
            // 2. 当前仅 BSP（CPU 0）运行且已通过 `_irq` 关闭本地中断，所有从核尚未唤醒，无并发内存访问；
            // 3. `boot::info()` 提供的启动信息已完整标记内核镜像、BSP 栈、页表及固件保留区，空闲 RAM 无其他使用者。
            unsafe { mm::initialize(boot::info()) };
            // SAFETY:
            // 1. CPU 拓扑表已在早期启动阶段建立，当前仅 CPU 0 执行且本地中断关闭；
            // 2. 上一行 `mm::initialize` 已完成页分配器初始化，所有从核尚未释放，不存在并发分配或栈访问。
            unsafe { cpu::prepare_secondary_stacks() }.expect("AP stack preparation failed");
        }
        // Step 3: 确认所有从核在唤醒前均处于离线状态，并记录各从核的硬件 ID、栈顶与 CpuLocal 地址。
        for id in 1..cpu::count() {
            assert!(!cpu::get(id)
                .online
                .load(core::sync::atomic::Ordering::Acquire));
            context.diagnostic(format_args!(
                "frozen-ap before id={} hardware={} stack={:#x} local={:#x}",
                id,
                cpu::get(id).hardware_id,
                cpu::get(id).stack_top(),
                cpu::get(id) as *const _ as usize
            ));
        }
        // Step 4: 发起从核唤醒流程；由于宿主机冻结了从核，预期在此调用内部等待超时并触发 Panic。
        // SAFETY:
        // 1. `BootInfo`、`CpuLocal`、内存管理子系统及各从核的独占启动栈均已在上方初始化完毕；
        // 2. 当前仅 CPU 0 运行，所有从核尚未释放，且未持有从核上线路径所需的任何自旋锁；
        // 3. 超时后函数直接触发未捕获 Panic 终止 VM，被冻结的从核栈不会被提前回收或复用。
        unsafe { cpu::start_secondaries() };
        panic!("frozen AP unexpectedly came online");
    }
}
