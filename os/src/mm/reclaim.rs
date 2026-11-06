//! 同步内存回收（Direct Reclaim）慢速路径与预算控制模块。
//!
//! ## 模块职责与架构定位
//! 本模块为普通可睡眠内核任务提供带严格预算上限的同步内存回收控制（`ReclaimAttempt`）。
//! - **上游调用者**：当物理页分配（`alloc_pages`）或文件系统缓存节点分配因内存不足（`NoMemory`）
//!   在快速路径（`NOWAIT`）失败时，由任务上下文显式构造 `ReclaimAttempt` 进入本慢速路径。
//! - **下游依赖**：调用页缓存回收入口（`crate::cache::reclaim_file`）扫描 LRU 队列、反映射（rmap）
//!   清理页表并回收干净的文件缓存页或空闲页表页，随后再次调用底层物理页分配器（`crate::mm::alloc_pages`）。
//!
//! ## 核心设计与预算状态机
//! 为避免直接回收在极端内存压力下陷入无限扫描或死锁，单次 `ReclaimAttempt` 受到三个维度的定额预算约束，
//! 且最多只允许执行一次写回等待与一次重试分配：
//!
//! ```text
//!   快速路径分配失败 (alloc_pages -> None / BlockError::NoMemory)
//!         │
//!         ▼
//!   构造 ReclaimAttempt::new()  [断言当前处于可睡眠任务上下文]
//!         │
//!         ▼
//!   执行 cache::reclaim_file(&mut attempt)
//!   ┌────────────────────────────────────────────────────────────┐
//!   │ • LRU 扫描预算: SCAN_BUDGET (64 页)                        │
//!   │ • 页面隔离预算: ISOLATE_BUDGET (16 页)                     │
//!   │ • 反向映射预算: RMAP_BUDGET (64 次检查)                    │
//!   │ • 写回等待限制: 仅 ReclaimMode::Normal 下最多等待 1 次     │
//!   └────────────────────────────────────────────────────────────┘
//!         │
//!         ├── 实际释放了文件页或页表页 (freed_file + freed_tables > 0)
//!         │         └──► 重试一次原分配请求
//!         └── 未能释放任何物理页
//!                   └──► 立即返回失败 (避免无进展死循环)
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `ReclaimAttempt` 是当前任务栈上的私有状态机，不跨线程共享，无需加锁。
//! - 进入本模块前不得持有自旋锁或关中断（由 `ReclaimAttempt::new` 中的 `assert_switchable` 强制检查）。
//! - 当调用者已持有文件系统锁（处于 `ReclaimMode::NoFs` 或 `ReclaimMode::NoIo` 上下文）时，
//!   `wait_writeback` 会拒绝触发或等待块设备写回，从而防止“文件系统锁 -> 内存回收 -> 文件系统写回”的递归死锁。

use crate::block::BlockError;
use crate::cache;
use crate::mm::{alloc_pages, OwnedPages};
use crate::task::ReclaimMode;

/// 单次回收请求扫描 LRU 链表的最大页面数预算（包含活跃页老化与候选页检查）。
const SCAN_BUDGET: usize = 64;
/// 单次回收请求从 LRU 链表中隔离（Isolate）出来的最大候选页面数预算。
const ISOLATE_BUDGET: usize = 16;
/// 单次回收请求执行反向映射（Reverse Mapping，含查验访问位与解除映射）的最大次数预算。
const RMAP_BUDGET: usize = 64;

/// 单次直接回收请求在执行过程中的各项操作与物理页释放统计。
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ReclaimStats {
    /// 已扫描的 LRU 队列页面总数（单位：页）。
    pub scanned: usize,
    /// 已从非活跃 LRU 队列成功隔离出的候选页面总数（单位：页）。
    pub isolated: usize,
    /// 已执行的反向映射（rmap）范围检查次数（单位：次）。
    pub rmap_checks: usize,
    /// 已从用户地址空间成功解除页表映射（unmap）的次数（单位：次）。
    pub unmapped: usize,
    /// 实际释放并归还给伙伴分配器的文件缓存物理页数量（单位：页）。
    pub freed_file: usize,
    /// 在解除映射过程中顺带回收的空闲用户页表物理页数量（单位：页）。
    pub freed_tables: usize,
    /// 回收期间向块设备提交回写的脏数据字节数（单位：字节）。
    pub submitted_bytes: usize,
    /// 回收期间已完成块设备回写的字节数（单位：字节）。
    pub completed_bytes: usize,
    /// 回收期间触发或等待的块设备 I/O 是否发生过错误。
    pub io_error: bool,
}

/// 单次直接内存回收上下文，跟踪剩余扫描预算、实际回收成效以及是否已等待过在途写回。
///
/// 高层操作（如分配目标物理页与分配中间级页表页）可共享同一个 `ReclaimAttempt` 实例，
/// 防止因多级重试叠加出无限循环的回收风暴。
pub struct ReclaimAttempt {
    /// 本次回收请求累计消耗的预算与释放统计。
    stats: ReclaimStats,
    /// 标记本次回收请求是否已经消耗过唯一一次等待在途写回（Writeback）的机会。
    waited: bool,
}

impl ReclaimAttempt {
    /// 创建新的直接内存回收上下文。
    ///
    /// 仅允许在可睡眠的普通任务上下文中调用（内部断言可调度状态）。
    pub(crate) fn new() -> Self {
        crate::sched::assert_switchable();
        Self {
            stats: ReclaimStats::default(),
            waited: false,
        }
    }

    /// 返回本次回收请求截至目前的累计统计快照。
    pub(crate) fn stats(&self) -> ReclaimStats {
        self.stats
    }

    /// 动态读取当前任务的内存回收模式（`Normal` / `NoFs` / `NoIo`）。
    ///
    /// 每次进入回收分支时实时读取任务状态，确保调用者在创建 `ReclaimAttempt` 后若新获取了文件系统锁
    /// （切换为 `NoFs`），回收逻辑能立即感知并禁止触发写回。
    pub(crate) fn mode(&self) -> ReclaimMode {
        ReclaimMode::current()
    }

    /// 尝试分配指定阶数（`order`）的物理页；若快速路径失败，则执行一轮内存回收并最多重试一次。
    pub(crate) fn allocate_pages(&mut self, order: usize, zeroed: bool) -> Option<OwnedPages> {
        if let Some(pages) = alloc_pages(order, zeroed) {
            return Some(pages);
        }
        self.reclaim().then(|| alloc_pages(order, zeroed)).flatten()
    }

    /// 执行一轮文件缓存回收，返回本轮是否实际向伙伴系统归还了至少一页物理内存（文件页或页表页）。
    pub(crate) fn reclaim(&mut self) -> bool {
        let before = self.stats.freed_file + self.stats.freed_tables;
        cache::reclaim_file(self);
        before < self.stats.freed_file + self.stats.freed_tables
    }

    /// 返回当前剩余可用的 LRU 扫描页面配额（`SCAN_BUDGET - scanned`）。
    pub(crate) fn scan_remaining(&self) -> usize {
        SCAN_BUDGET.saturating_sub(self.stats.scanned)
    }

    /// 尝试消耗 1 个单位的 LRU 扫描预算；若预算尚未耗尽则递增计数并返回 `true`。
    pub(crate) fn scan(&mut self) -> bool {
        if SCAN_BUDGET <= self.stats.scanned {
            return false;
        }
        self.stats.scanned += 1;
        true
    }

    /// 尝试消耗 1 个单位的页面隔离预算；若未达到 `ISOLATE_BUDGET` 上限则递增计数并返回 `true`。
    pub(crate) fn isolate(&mut self) -> bool {
        if ISOLATE_BUDGET <= self.stats.isolated {
            return false;
        }
        self.stats.isolated += 1;
        true
    }

    /// 检查当前是否仍有剩余的页面隔离预算（`isolated < ISOLATE_BUDGET`）。
    ///
    /// 当隔离预算耗尽但扫描预算仍有剩余时，扫描器不再隔离非活跃页，但仍可利用剩余扫描预算对活跃页进行老化降级。
    pub(crate) fn can_isolate(&self) -> bool {
        self.stats.isolated < ISOLATE_BUDGET
    }

    /// 尝试消耗 1 个单位的反向映射（rmap）检查预算；检查清脏（mkclean）与解除映射（unmap）共享该配额。
    pub(crate) fn rmap(&mut self) -> bool {
        if RMAP_BUDGET <= self.stats.rmap_checks {
            return false;
        }
        self.stats.rmap_checks += 1;
        true
    }

    /// 尝试申请唯一一次等待在途写回（Writeback）完成的许可。
    ///
    /// 仅当当前任务处于 `ReclaimMode::Normal` 且本请求尚未等待过写回时返回 `true`；
    /// 在 `NoFs` 或 `NoIo` 模式下始终返回 `false`，避免递归重入文件系统或块设备层引发死锁。
    pub(crate) fn wait_writeback(&mut self) -> bool {
        if self.waited || self.mode() != ReclaimMode::Normal {
            return false;
        }
        self.waited = true;
        true
    }

    /// 将下层回收步骤产生的解除映射数、释放页数及回写 I/O 统计累加到当前回收上下文中。
    pub(crate) fn account(&mut self, stats: ReclaimStats) {
        self.stats.unmapped += stats.unmapped;
        self.stats.freed_file += stats.freed_file;
        self.stats.freed_tables += stats.freed_tables;
        self.stats.submitted_bytes += stats.submitted_bytes;
        self.stats.completed_bytes += stats.completed_bytes;
        self.stats.io_error |= stats.io_error;
    }

    /// 执行带 `NoMemory` 自动回收重试的资源准备闭包 `prepare`。
    ///
    /// 若首次调用返回 `Err(BlockError::NoMemory)`（此时闭包内部已建半成品均已析构释放），
    /// 则触发一轮内存回收；仅当回收实际释放了物理页时，才重新调用一次 `prepare`。
    pub(crate) fn prepare<T>(
        &mut self,
        mut prepare: impl FnMut() -> Result<T, BlockError>,
    ) -> Result<T, BlockError> {
        match prepare() {
            Err(BlockError::NoMemory) if self.reclaim() => prepare(),
            result => result,
        }
    }
}
