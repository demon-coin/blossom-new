//! 内核任务执行、等待同步与跨核内存归属测试组的公共同步等待与资源对账底座。
//!
//! # 模块职责与架构定位
//! 本模块作为 `execution_core` 测试子树的根模块，统管三个核心执行测试子模块，并向它们提供统一的有界条件等待与系统静止态资源对账原语：
//! - **下属测试子模块**：
//!   - `context`：普通内核任务的上下文切换、跨核迁移及致命异常（Fatal）恢复测试。
//!   - `wait`：等待队列（`WaitQueue`）与睡眠互斥锁（`SleepMutex`）的阻塞唤醒与竞争测试。
//!   - `memory`：物理页块与全局堆对象在本核及跨核迁移场景下的分配、内容保持与异核释放测试。
//! - **上游调用者**：`ktest` 注册中心调用 [`anchors`] 收集本子树下全部 6 个测试套件的静态描述符；各子模块的测试夹具（Fixture）在 `set_up`、`quiesce` 与 `tear_down` 阶段调用 [`until`]、[`baseline`] 与 [`reclaimed`]。
//! - **下游依赖**：协同 `klog`、`workqueue`、`cpu`、`sched`、`time` 与 `mm` 子系统，排空异步延迟回收队列并采样全系统内存与工作队列元数据统计。
//!
//! # 核心设计与静止态资源对账流程
//! 内核中任务的退出栈回收、SLUB 每 CPU 缓存页及伙伴系统单页缓存（PCP）存在异步或延迟归还路径。
//! 为避免并发后台活动造成误报，[`baseline`] 与 [`reclaimed`] 按以下四步流水线将系统收敛至静止态后再逐字段核对：
//!
//! ```text
//!   [测试用例及派生子任务全部 join 退出]
//!                    │
//!                    ▼
//!   Step 1: 等待常驻后台服务进入阻塞休眠
//!           klog::wait_service_blocked() + workqueue::wait_services_blocked()
//!                    │
//!                    ▼
//!   Step 2: 等待除当前核外的所有在线 CPU 进入 idle 状态
//!           确保远核调度器已完成上一个切换出栈任务的尾部清理
//!                    │
//!                    ▼
//!   Step 3: 排空各级内存分配器本地缓存
//!           mm::shrink_all() (归还空闲 Slab 页) + mm::drain_all_pages() (排空各核 PCP)
//!                    │
//!                    ▼
//!   Step 4: 采样 (mm::MemoryStats, workqueue::MetadataStats) 并与初始基线逐字段比对
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`until`]、[`baseline`] 与 [`reclaimed`] 内部会调用 `sched::yield_now()` 主动让出 CPU 或发起跨核缓存排空 IPI，**严禁**在持有自旋锁（`SpinLock`）、抢占守卫（`PreemptGuard`）或关中断守卫（`IrqSaveGuard`）的上下文中调用。

mod context;
mod memory;
mod wait;

use crate::{cpu, mm, sched};
use ::ktest::{TestError, TestResult};
use core::sync::atomic::Ordering;

/// 执行核心测试组的系统资源基线快照，由物理内存六域统计（[`mm::MemoryStats`]）与工作队列元数据统计组合而成。
pub(crate) type Baseline = (mm::MemoryStats, crate::workqueue::testing::MetadataStats);

/// 在可调度任务上下文中轮询等待 `predicate` 成立（最长等待 5 秒），每次未满足时主动调用 `sched::yield_now()` 让出 CPU；若超时则返回携带 `reason` 的测试错误。
pub(crate) fn until(mut predicate: impl FnMut() -> bool, reason: &'static str) -> TestResult {
    let end = crate::time::now()
        .checked_add(crate::time::Duration::from_secs(5))
        .ok_or(TestError("execution deadline overflow"))?;
    while !predicate() {
        if end <= crate::time::now() {
            return Err(TestError(reason));
        }
        sched::yield_now();
    }
    Ok(())
}

/// 等待后台日志与工作队列服务进入阻塞态、其他在线 CPU 全部进入空闲态，并排空 SLUB 与每 CPU 页缓存后，采集当前内存与工作队列元数据基线。
pub(crate) fn baseline() -> Result<Baseline, TestError> {
    // Step 1: 等待常驻后台日志线程与工作队列工作者线程处理完积压任务并进入阻塞睡眠。
    crate::klog::testing::wait_service_blocked();
    crate::workqueue::testing::wait_services_blocked();
    // Step 2: 等待除当前执行核以外的所有在线 CPU 进入 idle 循环，确保远核调度切换的收尾工作全部结束。
    until(
        || {
            (0..cpu::count())
                .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
        },
        "remote scheduler resource tail",
    )?;
    // Step 3: 收缩全局 SLUB 空闲缓存页并排空各核 PCP 单页缓存，消除本地缓存滞留带来的统计波动。
    mm::shrink_all();
    mm::drain_all_pages();
    Ok((mm::stats(), crate::workqueue::testing::metadata_stats()))
}

/// 在测试用例持有的全部临时对象释放后重新采集系统静止态快照，逐项核对物理内存统计与工作队列元数据是否完整恢复至 `expected` 基线。
pub(crate) fn reclaimed(expected: Baseline) -> TestResult {
    let actual = baseline()?;
    if actual.0 != expected.0 {
        return Err(TestError("temporary execution memory did not converge"));
    }
    if actual.1 != expected.1 {
        return Err(TestError("temporary execution metadata did not converge"));
    }
    Ok(())
}

/// 汇总并返回 `execution_core` 子树下全部 6 个测试套件的静态注册槽引用，防止链接器裁剪测试描述符。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 6] {
    [
        context::context::__ktest_anchor(),
        context::migration::__ktest_anchor(),
        context::fatal::__ktest_anchor(),
        wait::queue::__ktest_anchor(),
        wait::mutex::__ktest_anchor(),
        memory::memory::__ktest_anchor(),
    ]
}
