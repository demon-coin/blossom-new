//! 多核（SMP）并发环境下的物理页跨核释放、PCP 远端直接排空、SLUB 共享空闲链交接与 `Arc` 跨核析构单元测试。
//!
//! ## 模块职责与架构定位
//! 本模块验证物理页分配器（`pcp`/`buddy`）、SLUB 小对象分配器（`slub`）及内核堆引用计数（`Arc`）
//! 在多 CPU 绑定任务并发执行时的跨核所有权转移与缓存同步语义，并向 `mm::ktest::irq` 导出 [`SmpFixture`] 与 [`State`]：
//! - **上游驱动者**：`ktest` 测试运行器调用 `pages`、`objects`、`arc_ownership` 套件，以及 `irq::IrqFixture` 组合复用。
//! - **被测与下游依赖**：通过 `ktest::spawn_on` 在各在线 CPU 上派生绑定任务，调用 `mm::alloc_pages`、`mm::drain_all_pages`、
//!   `pcp` 水位与计数查询、`slub` 快照接口以及 `pages::check_try_lock_restore`。
//!
//! ## 核心并发机制与 SLUB 跨核空闲链交接
//! 1. **零阶物理页跨核释放与直接排空**：CPU A 分配的零阶物理页移交 CPU B 释放时，只进入释放方（CPU B）的本地 PCP 缓存；
//!    控制核调用 `drain_all_pages()` 可直接加锁排空所有远端 CPU 的 PCP 缓存至伙伴系统，无需依赖跨核中断（IPI）。
//! 2. **SLUB `shared` 空闲链到 `local` 空闲链的接管流转**：
//!
//! ```text
//! CPU0 (Active Slab 拥有者)                        CPU1 (远端释放者)
//! ────────────────────────                        ──────────────────
//! 1. 填满整个 Active(0) Slab (local_count == 0)
//! 2. 将 2 个对象移交槽位 64..66 ─────────────────► 3. 异核释放 2 个对象：
//!                                                     写入头页 SlabState::shared 链 (inuse = count - 2)
//! 4. 首次再分配：检测到 local 为空而 shared 非空 ◄─────┘
//!    ├─ 一次性将整条 shared 链搬入 CPU0 local 链 (shared 清零, inuse 恢复为 count)
//!    └─ 弹出第 1 个对象后，local_count == 1
//! 5. 第二次分配：直接无锁/本核锁命中 CPU0 local 链 (local_count 归零)，可与下一轮 CPU1 释放并行执行
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 测试控制结构 [`State`] 中的槽位锁（`pages[i]`、`objects[i]`、`payload`）均为叶级短临界区自旋锁：
//!   **严禁在持有上述槽位锁期间嵌套获取另一个槽位锁，或调用内存释放（`drop`）触发底层分配器自旋锁**。
//!   所有释放路径均先通过 `.lock().take()` 摘出所有权并立即释放槽位锁，随后在锁外执行实际内存归还。
//! - 所有子任务完成后调用 `State::park` 驻留等待，直到 `SmpFixture::quiesce` 置位 `stop` 并由框架完成全部子任务 `join` 退出后，
//!   才在 `tear_down` 中回收残留资源并核对物理内存六域统计基线。

use crate::mm::ktest::{free, recovered, settled, SlabObject, CLASS};
use crate::{
    arch, cpu,
    mm::{self, buddy, page::SlabLocation, pcp, slub, MemoryStats, OwnedPages},
    sync::SpinLock,
};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use ktest::{Fixture, TestContext, TestError, TestResult};

/// 系统支持的最大 CPU 核心槽位数（与 `boot::MAX_CPUS` 对齐）。
const MAX: usize = crate::boot::MAX_CPUS;
/// 每个在线 CPU 在 [`State::pages`] 与 [`State::identities`] 中独占分配的物理页槽位跨度（每核 32 槽）。
pub(super) const WORK: usize = 32;

/// 多核内存测试的共享控制与资源托管状态块。
pub(crate) struct State {
    /// 协作停止标志：由 `SmpFixture::quiesce` 置为 `true`，唤醒驻留在 `wait`/`park` 中的所有子任务退出。
    pub(crate) stop: AtomicBool,
    /// 已启动并在目标 CPU 上就绪的子任务计数。
    pub(crate) ready: AtomicUsize,
    /// 控制者向各核子任务广播的当前测试阶段号（通过 Release/Acquire 构成跨核阶段屏障）。
    pub(crate) phase: AtomicUsize,
    /// 串行轮流执行阶段中当前获准动作的目标 CPU 编号。
    pub(crate) turn: AtomicUsize,
    /// 并发排空流测试中，控制核已完成至少一次 `drain_all_pages` 扫描的阶段纪元号。
    drain_epoch: AtomicUsize,
    /// 每个 CPU 子任务上报自身已完成的最大阶段号。
    pub(crate) done: [AtomicUsize; MAX],
    /// 按 `cpu_id * WORK + index` 分区托管的物理页块所有权槽位表，每个槽位受独立自旋锁保护。
    pub(crate) pages: Vec<SpinLock<Option<OwnedPages>>>,
    /// 与 `pages` 一一对应的物理页首地址（字节物理地址）快照表，用于跨核去重与身份校验。
    identities: Vec<AtomicUsize>,
    /// 托管跨核测试中存活 SLUB 对象所有权的固定槽位数组（前段供主核持有，64..70 供跨核释放交接）。
    pub(crate) objects: [SpinLock<Option<SlabObject>>; 96],
    /// 主核向远端工作核发出的轮次指令号。
    pub(crate) command: AtomicUsize,
    /// 远端工作核向主核回传的轮次完成确认号。
    pub(crate) acknowledgement: AtomicUsize,
    /// `Payload` 实例析构函数实际被调用的累计次数。
    drops: AtomicUsize,
    /// 执行最后一个 `Arc<Payload>` 强引用释放并触发 `Payload::drop` 的物理 CPU 编号。
    drop_cpu: AtomicUsize,
    /// 托管跨核引用计数测试共享 `Arc<Payload>` 的自旋锁槽位。
    payload: SpinLock<Option<Arc<Payload>>>,
}
impl State {
    /// 按当前在线 CPU 数量在堆上精确预分配每核物理页槽位表与身份表，构造初始多核控制状态。
    pub(crate) fn new() -> Result<Self, TestError> {
        let slots = cpu::count() * WORK;
        let mut pages = Vec::new();
        pages
            .try_reserve_exact(slots)
            .map_err(|_| TestError("NoMemory: SMP page slots"))?;
        pages.resize_with(slots, || SpinLock::new(None));

        let mut identities = Vec::new();
        identities
            .try_reserve_exact(slots)
            .map_err(|_| TestError("NoMemory: SMP page identities"))?;
        identities.resize_with(slots, || AtomicUsize::new(usize::MAX));
        Ok(Self {
            stop: AtomicBool::new(false),
            ready: AtomicUsize::new(0),
            phase: AtomicUsize::new(0),
            turn: AtomicUsize::new(0),
            drain_epoch: AtomicUsize::new(0),
            done: [const { AtomicUsize::new(0) }; MAX],
            pages,
            objects: [const { SpinLock::new(None) }; 96],
            identities,
            command: AtomicUsize::new(0),
            acknowledgement: AtomicUsize::new(0),
            drops: AtomicUsize::new(0),
            drop_cpu: AtomicUsize::new(usize::MAX),
            payload: SpinLock::new(None),
        })
    }

    /// 在普通可抢占任务中协作等待条件 `ready` 成立；若收到停止信号或超过 10 秒硬件计数上限则返回错误。
    pub(crate) fn wait(
        &self,
        token: &crate::ktest::ChildToken,
        mut ready: impl FnMut() -> bool,
    ) -> TestResult {
        let end = arch::read_counter().wrapping_add(arch::counter_frequency() * 10);
        while !ready() {
            if self.stop.load(Ordering::Acquire) || token.stop_requested() {
                return Err(TestError("memory participant stopped"));
            }
            if end <= arch::read_counter() {
                return Err(TestError("memory participant timeout"));
            }
            crate::sched::yield_now();
        }
        Ok(())
    }

    /// 使已完成本阶段测试工作的子任务让出 CPU 驻留等待，直至 `quiesce` 阶段发出 `stop` 停止通知后再退出栈帧。
    pub(crate) fn park(&self, token: &crate::ktest::ChildToken) -> TestResult {
        while !self.stop.load(Ordering::Acquire) && !token.stop_requested() {
            crate::sched::yield_now();
        }
        Ok(())
    }

    /// 在所有参与核静止的同步点上，校验 `first..cpu::count()` 各核前 `count` 个槽位持有的物理页地址两两唯一且不重叠。
    fn unique_pages(&self, first: usize, count: usize) {
        assert!(count <= WORK);
        for id in first..cpu::count() {
            for index in 0..count {
                let pfn = self.pages[id * WORK + index]
                    .lock()
                    .as_ref()
                    .unwrap()
                    .phys_addr();
                for other in first..=id {
                    let previous = if other == id { index } else { count };
                    for before in 0..previous {
                        assert_ne!(
                            self.identities[other * WORK + before].load(Ordering::Relaxed),
                            pfn
                        );
                    }
                }
                self.identities[id * WORK + index].store(pfn, Ordering::Relaxed);
            }
        }
    }

    /// 分配一个 4096B SLUB 对象并立即存入 `objects[index]` 托管，随后锁外校验其虚拟地址与所有其余存活对象互不相同。
    pub(crate) fn allocate_object(&self, index: usize) {
        let object = SlabObject::allocate();
        let mut slot = self.objects[index].lock();
        assert!(slot.is_none());
        *slot = Some(object);
        let address = slot.as_ref().unwrap().0.as_ptr() as usize;
        drop(slot);
        for (other, slot) in self.objects.iter().enumerate() {
            if other != index {
                if let Some(object) = slot.lock().as_ref() {
                    assert_ne!(object.0.as_ptr() as usize, address);
                }
            }
        }
    }

    /// 读取 `objects[index]` 槽位中存活对象的非空裸指针；调用方须在同步点保证远端核心此刻不会并发取走该槽位。
    pub(crate) fn pointer(&self, index: usize) -> core::ptr::NonNull<u8> {
        self.objects[index].lock().as_ref().unwrap().0
    }

    /// 先校验 `objects[index]` 中对象的数据与首尾签名完好，再将其从槽位摘出并在槽位自旋锁外执行释放。
    pub(crate) fn release_object(&self, index: usize) {
        self.objects[index].lock().as_ref().unwrap().check();
        let object = self.objects[index].lock().take();
        drop(object);
    }
}

/// 管理多核绑定子任务生命周期、异常清理与物理内存六域基线核账的测试夹具。
pub(crate) struct SmpFixture {
    /// 用例启动前刷回并排空缓存后记录的物理内存六域统计基线。
    baseline: MemoryStats,
    /// 夹具与所有子任务共享的多核控制与资源托管块。
    pub(crate) state: Arc<State>,
}
impl Fixture for SmpFixture {
    /// 采集初始物理内存六域基线，并可失败地（fallible）分配多核共享状态块。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled();
        let state =
            Arc::try_new(State::new()?).map_err(|_| TestError("NoMemory: SMP memory controls"))?;
        Ok(Self { baseline, state })
    }

    /// 置位协作停止标志唤醒驻留子任务；随后由 `ktest` 框架统一请求停止并阻塞 `join` 全部子任务。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.state.stop.store(true, Ordering::Release);
        Ok(())
    }

    /// 在全部子任务已退出且归还 `Arc<State>` 引用后，清空所有托管槽位与控制块，并校验六域统计完全恢复初始基线。
    fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
        if Arc::strong_count(&self.state) != 1 {
            return Err(TestError("memory SMP participant retains controls"));
        }
        for slot in &self.state.pages {
            let pages = slot.lock().take();
            drop(pages);
        }
        for slot in &self.state.objects {
            let object = slot.lock().take();
            drop(object);
        }

        let payload = self.state.payload.lock().take();
        drop(payload);
        let baseline = self.baseline;
        drop(self);
        recovered(context, baseline);
        Ok(())
    }
}
impl SmpFixture {
    /// 在指定物理核心 `id` 上派生绑定子任务，核验运行核心身份并递增 `ready` 计数后执行任务闭包 `body`。
    pub(crate) fn spawn(
        &self,
        id: usize,
        body: impl FnOnce(Arc<State>, crate::ktest::ChildToken) -> TestResult + Send + 'static,
    ) -> TestResult {
        let state = Arc::clone(&self.state);
        crate::ktest::spawn_on(id, move |token| {
            assert_eq!(cpu::current().id, id);
            state.ready.fetch_add(1, Ordering::Release);
            body(state, token)
        })?;
        Ok(())
    }

    /// 控制任务轮询等待条件 `ready` 成立，期间主动 `yield_now` 让出 CPU 以允许同核子任务调度运行（15 秒超时）。
    pub(crate) fn until(&self, mut ready: impl FnMut() -> bool) -> TestResult {
        let end = arch::read_counter().wrapping_add(arch::counter_frequency() * 15);
        while !ready() {
            if end <= arch::read_counter() {
                return Err(TestError("memory controller timeout"));
            }
            crate::sched::yield_now();
        }
        Ok(())
    }

    /// 等待所有在线 CPU（`0..cpu::count()`）上的子任务均上报完成指定阶段 `phase`。
    fn all_done(&self, phase: usize) -> TestResult {
        self.until(|| {
            self.state.done[..cpu::count()]
                .iter()
                .all(|d| d.load(Ordering::Acquire) == phase)
        })
    }
}

/// 多核环境下的零阶物理页跨核释放归属、每核 PCP 水位、远端直接排空与每核自旋锁中断恢复测试套件。
#[ktest::suite(fixture = SmpFixture, tags("mm", "smp", "pcp"), timeout_ms = 30000)]
mod pages {
    use super::*;
    /// 验证跨核传递的零阶物理页在最后一个持有者析构时，仅归入实际执行释放操作的当前 CPU 的 PCP 缓存。
    ///
    /// - **测试目标**：确认物理页跨核流转无重复分配且数据可见性正确，析构释放严格进入当前运行核而非原分配核的 PCP。
    /// - **场景步骤**：第 1 阶段每个在线 CPU 各自独立分配 8 张零阶页，在页首写入核心与槽位标识并存入共享槽位，控制者排空全核 PCP 并校验全部存活页物理地址唯一；第 2 阶段各核按 `turn` 顺序依次接管并释放前驱核心（`(id + count - 1) % count`）分配的 8 张页，逐张校验内容与本核 PCP 缓存增量。
    /// - **预期结果**：所有存活页互不重复，跨核读取页首数据准确，每次释放精确计入当前释放核的 PCP（满 `high` 自动扣减 `batch`），最终排空后全局空闲页完全恢复。
    #[test]
    fn final_drop_cpu(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        for id in 0..cpu::count() {
            f.spawn(id, |state, token| {
                let id = cpu::current().id;
                // Step 1: 等待阶段 1 开启，在本核分配 8 张零阶页，写入特征字后发布至本核所属槽位。
                state.wait(&token, || state.phase.load(Ordering::Acquire) == 1)?;
                for index in 0..8 {
                    let mut pages = mm::alloc_pages(0, false).unwrap();
                    state.identities[id * WORK + index].store(pages.phys_addr(), Ordering::Relaxed);
                    // SAFETY:
                    // 1. `pages` 刚由本核 `alloc_pages(0, false)` 独占取得，尚未存入共享表，无任何并发访问者；
                    // 2. 4 KiB 物理页直接映射首地址天然满足 `usize` 对齐，写入单个 `usize` 后经下方锁与 `done[id]` Release 屏障同步给后续读取核。
                    unsafe { pages.as_mut_ptr().cast::<usize>().write(id * 8 + index) };
                    *state.pages[id * WORK + index].lock() = Some(pages);
                }
                state.done[id].store(1, Ordering::Release);
                // Step 2: 轮到本核（turn == id）时，依次取出前驱邻核分配的 8 张页校验内容并在本核释放，核对本核 PCP 计数变化。
                state.wait(&token, || {
                    state.phase.load(Ordering::Acquire) == 2
                        && state.turn.load(Ordering::Acquire) == id
                })?;
                let (batch, high) = pcp::parameters();
                let mut expected = 0;
                assert_eq!(pcp::cpu_cached_pages(id), 0);
                let source = (id + cpu::count() - 1) % cpu::count();
                for index in 0..8 {
                    let mut slot = state.pages[source * WORK + index].lock();
                    let pages = slot.as_mut().unwrap();
                    assert_eq!(
                        pages.phys_addr(),
                        state.identities[source * WORK + index].load(Ordering::Relaxed)
                    );
                    // SAFETY:
                    // 1. 源核心已在阶段 1 完成首字初始化并通过 `done` / `phase` Acquire-Release 屏障完成同步，此后不再访问该页；
                    // 2. 当前持有 `state.pages[source * WORK + index]` 自旋锁且尚未释放 `pages`，页首 `usize` 对齐且内存有效。
                    assert_eq!(
                        unsafe { pages.as_mut_ptr().cast::<usize>().read() },
                        source * 8 + index
                    );
                    let pages = slot.take();
                    drop(slot);
                    drop(pages);
                    expected += 1;
                    if high <= expected {
                        expected -= batch;
                    }
                    assert_eq!(pcp::cpu_cached_pages(id), expected);
                }
                state.turn.store(id + 1, Ordering::Release);
                state.done[id].store(2, Ordering::Release);
                state.park(&token)
            })?;
        }

        // 控制流：等待全核就绪 -> 推进阶段 1 并校验跨核去重 -> 推进阶段 2 串行异核释放 -> 排空复核总账。
        f.until(|| f.state.ready.load(Ordering::Acquire) == cpu::count())?;
        mm::drain_all_pages();
        let initial = free();
        f.state.phase.store(1, Ordering::Release);
        f.all_done(1)?;
        f.state.unique_pages(0, 8);
        assert_eq!(free(), initial - 8 * cpu::count());
        mm::drain_all_pages();
        assert_eq!(pcp::cached_pages(), 0);

        f.state.phase.store(2, Ordering::Release);
        f.all_done(2)?;
        assert!(f.state.pages.iter().all(|p| p.lock().is_none()));
        mm::drain_all_pages();
        assert_eq!(free(), initial);
        Ok(())
    }
    /// 验证系统中每一个在线 CPU 的本地 PCP 缓存独立达到 `high` 水位时，均精确向伙伴系统批量归还 `batch` 张页。
    ///
    /// - **测试目标**：确认非启动核（AP）与启动核（BSP）各自的 `Pageset` 水位参数与批量排空逻辑完全一致。
    /// - **场景步骤**：在派生子任务前预先读取 `(batch, high)` 常量；各在线 CPU 按 `turn` 顺序串行执行：分配 `high` 张零阶页、排空多余缓存、先释放 `high - 1` 张观察滞留，再释放第 `high` 张观察批量归还。
    /// - **预期结果**：每个 CPU 在释放 `high - 1` 张时缓存计数均为 `high - 1`，释放第 `high` 张后本核缓存降为 `high - batch` 且伙伴系统空闲页增加 `batch`。
    #[test]
    fn every_cpu_high(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        let (batch, high) = pcp::parameters();
        for id in 0..cpu::count() {
            f.spawn(id, move |state, token| {
                let id = cpu::current().id;
                state.wait(&token, || {
                    state.phase.load(Ordering::Acquire) == 1
                        && state.turn.load(Ordering::Acquire) == id
                })?;
                mm::drain_all_pages();
                let initial = free();
                for index in 0..high {
                    *state.pages[id * WORK + index].lock() =
                        Some(mm::alloc_pages(0, false).unwrap());
                }
                mm::drain_all_pages();
                assert_eq!(buddy::free_pages(), initial - high);

                for index in 0..high - 1 {
                    let pages = state.pages[id * WORK + index].lock().take();
                    drop(pages);
                }
                assert_eq!(pcp::cpu_cached_pages(id), high - 1);
                assert_eq!(buddy::free_pages(), initial - high);

                let pages = state.pages[id * WORK + high - 1].lock().take();
                drop(pages);
                assert_eq!(pcp::cpu_cached_pages(id), high - batch);
                assert_eq!(buddy::free_pages(), initial - high + batch);
                mm::drain_all_pages();
                assert_eq!(free(), initial);
                state.turn.store(id + 1, Ordering::Release);
                state.done[id].store(1, Ordering::Release);
                state.park(&token)
            })?;
        }
        f.until(|| f.state.ready.load(Ordering::Acquire) == cpu::count())?;
        f.state.phase.store(1, Ordering::Release);
        f.all_done(1)
    }
    /// 验证各个远端 CPU 缓存零阶页并静止后，控制核调用 `drain_all_pages()` 可直接排空所有远端 PCP 缓存而无需发送 IPI。
    ///
    /// - **测试目标**：确认 `pcp::drain_all_pages` 能够跨核安全加锁回收所有 CPU `Pageset` 中的缓存空闲页。
    /// - **场景步骤**：每个在线 CPU 各自分配并立即释放 `batch` 张零阶页使其留存在各自本地 PCP 中，随后进入等待屏障；控制核验证每核缓存均为 `batch` 后调用一次 `drain_all_pages()`。
    /// - **预期结果**：单次 `drain_all_pages()` 后全核 `pcp::cached_pages()` 归零，所有页完整回到伙伴系统空闲链表。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_cache_drain(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        let (batch, _) = pcp::parameters();
        for id in 0..cpu::count() {
            f.spawn(id, move |state, token| {
                let id = cpu::current().id;
                state.wait(&token, || state.phase.load(Ordering::Acquire) == 1)?;
                for index in 0..batch {
                    *state.pages[id * WORK + index].lock() =
                        Some(mm::alloc_pages(0, false).unwrap());
                }
                for index in 0..batch {
                    let pages = state.pages[id * WORK + index].lock().take();
                    drop(pages);
                }
                assert_eq!(pcp::cpu_cached_pages(id), batch);
                state.done[id].store(1, Ordering::Release);
                state.wait(&token, || state.phase.load(Ordering::Acquire) == 2)?;
                state.park(&token)
            })?;
        }

        f.until(|| f.state.ready.load(Ordering::Acquire) == cpu::count())?;
        mm::drain_all_pages();
        let initial = free();
        f.state.phase.store(1, Ordering::Release);
        f.all_done(1)?;
        for id in 0..cpu::count() {
            assert_eq!(pcp::cpu_cached_pages(id), batch);
        }
        assert_eq!(free(), initial);

        mm::drain_all_pages();
        assert_eq!(pcp::cached_pages(), 0);
        assert_eq!(buddy::free_pages(), initial);
        f.state.phase.store(2, Ordering::Release);
        Ok(())
    }
    /// 验证从核（AP）连续多轮并发分配、释放零阶页的同时，主核高频并发调用 `drain_all_pages()` 时的锁竞争回退与账本守恒。
    ///
    /// - **测试目标**：检验 `pcp` 在与远端 `drain_all_pages` 发生 `Pageset` 自旋锁竞争时的伙伴系统直通回退路径及并发安全性。
    /// - **场景步骤**：在所有从核上循环 16 轮，奇数阶段各分配 `high` 张零阶页，偶数阶段逐张释放；与此同时主核在 `until` 循环中持续调用 `drain_all_pages()` 制造跨核锁竞争，并在每阶段收敛停点校验存活页唯一性与伙伴系统账本。
    /// - **预期结果**：16 轮高强度并发分配/释放与跨核排空全程无死锁、无重复发放页帧，每轮停点排空后伙伴系统空闲页数精确符合公式。
    #[test]
    #[ktest(min_cpus = 2)]
    fn parallel_drain_stream(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        for id in 1..cpu::count() {
            f.spawn(id, |state, token| {
                let id = cpu::current().id;
                let (_, high) = pcp::parameters();
                for round in 0..16 {
                    let phase = 1 + round * 2;
                    state.wait(&token, || {
                        state.phase.load(Ordering::Acquire) == phase
                            && phase <= state.drain_epoch.load(Ordering::Acquire)
                    })?;
                    for index in 0..high {
                        *state.pages[id * WORK + index].lock() =
                            Some(mm::alloc_pages(0, false).unwrap());
                    }
                    state.done[id].store(phase, Ordering::Release);
                    state.wait(&token, || {
                        state.phase.load(Ordering::Acquire) == phase + 1
                            && phase < state.drain_epoch.load(Ordering::Acquire)
                    })?;
                    for index in 0..high {
                        let pages = state.pages[id * WORK + index].lock().take();
                        drop(pages);
                        assert!(pcp::cpu_cached_pages(id) < high);
                    }
                    state.done[id].store(phase + 1, Ordering::Release);
                }
                state.park(&token)
            })?;
        }

        f.until(|| f.state.ready.load(Ordering::Acquire) == cpu::count() - 1)?;
        mm::drain_all_pages();
        let initial = free();
        let (_, high) = pcp::parameters();
        for round in 0..16 {
            let phase = 1 + round * 2;
            f.state.phase.store(phase, Ordering::Release);
            f.until(|| {
                mm::drain_all_pages();
                f.state.drain_epoch.store(phase, Ordering::Release);
                for id in 0..cpu::count() {
                    assert!(pcp::cpu_cached_pages(id) < high);
                }
                f.state.done[1..cpu::count()]
                    .iter()
                    .all(|d| d.load(Ordering::Acquire) == phase)
            })?;
            f.state.unique_pages(1, high);
            mm::drain_all_pages();
            assert_eq!(pcp::cached_pages(), 0);
            assert_eq!(buddy::free_pages(), initial - high * (cpu::count() - 1));

            f.state.phase.store(phase + 1, Ordering::Release);
            f.until(|| {
                mm::drain_all_pages();
                f.state.drain_epoch.store(phase + 1, Ordering::Release);
                f.state.done[1..cpu::count()]
                    .iter()
                    .all(|d| d.load(Ordering::Acquire) == phase + 1)
            })?;
            mm::drain_all_pages();
            assert_eq!(pcp::cached_pages(), 0);
            assert_eq!(buddy::free_pages(), initial);
        }
        Ok(())
    }
    /// 验证每个在线物理 CPU 在各自独立的绑定任务栈上执行 `SpinLock::try_lock` 时，开/关中断状态恢复均准确无误。
    ///
    /// - **测试目标**：确认所有在线核心（含各从核 AP）的中断保存与恢复 CSR 操作与 `SpinLock` 守卫语义一致。
    /// - **场景步骤**：在每个在线 CPU 上派生绑定子任务，各自在本地栈上调用 [`crate::mm::ktest::pages::check_try_lock_restore`]。
    /// - **预期结果**：所有在线核心均通过开中断与关中断两种初始状态下的自旋锁中断恢复断言。
    #[test]
    #[case("enabled", true)]
    #[case("disabled", false)]
    fn every_cpu_lock_restore(
        _: &mut TestContext<'_>,
        f: &mut SmpFixture,
        enabled: bool,
    ) -> TestResult {
        for id in 0..cpu::count() {
            f.spawn(id, move |state, token| {
                crate::mm::ktest::pages::check_try_lock_restore(enabled);
                state.done[id].store(1, Ordering::Release);
                state.park(&token)
            })?;
        }
        f.all_done(1)
    }
}

/// 多核环境下的 SLUB 跨核释放共享空闲链（`shared`）收集与拥有核本地空闲链（`local`）批量接管测试套件。
#[ktest::suite(fixture = SmpFixture, tags("mm", "smp", "slub"), min_cpus = 2, timeout_ms = 30000)]
mod objects {
    use super::*;
    /// 验证从核异核释放对象只挂入头页 `shared` 链表，且拥有核下次分配时一次性将 `shared` 搬入本核 `local` 空闲链。
    ///
    /// - **测试目标**：检验 SLUB 跨核无竞争释放路径（`SlabState::shared`）与本核活动块重填路径（`shared -> local`）在多轮交错下的状态转换。
    /// - **场景步骤**：CPU0 分配填满整个活动 Slab（`local_count == 0`），将前 6 个对象转入共享交接槽 `64..70`；分 3 轮通知 CPU1 每轮异核释放 2 个对象，CPU0 收到回执后先核验该块仍为 `Active(0)`、`local_count == 0` 且 `shared != 0`，再分配第 1 个对象触发接管，随后立即通知 CPU1 开始下一轮异核释放并与本核第 2 个对象的 `local` 命中并行执行。
    /// - **预期结果**：每轮 CPU1 释放后 `inuse == count - 2` 且 `shared` 非空；CPU0 分配第 1 个对象后 `shared` 清零、`local_count == 1`；分配第 2 个对象命中刚释放的另一槽位且 `local_count` 归零；3 轮结束后全部对象安全释放且无内存泄漏。
    #[test]
    fn shared_handoff(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        f.spawn(1, |state, token| {
            for round in 1..=3 {
                state.wait(&token, || state.command.load(Ordering::Acquire) == round)?;
                state.release_object(64 + (round - 1) * 2);
                state.release_object(65 + (round - 1) * 2);
                state.acknowledgement.store(round, Ordering::Release);
            }
            state.done[1].store(1, Ordering::Release);
            state.park(&token)
        })?;
        f.spawn(0, |state, token| {
            // Step 1: 在 CPU0 上填满一整个 Active Slab，并将前 6 个对象放入交接槽 64..70 供 CPU1 分三轮异核释放。
            let count = slub::geometry(CLASS).objects;
            assert!(6 < count);
            for index in 0..count {
                state.allocate_object(index);
            }
            let anchor = state.pointer(6);
            let head = slub::slab_snapshot(anchor).head_pfn;
            assert_eq!(slub::cpu_snapshot(CLASS, 0).local_count, 0);
            let identities: [usize; 6] =
                core::array::from_fn(|i| state.pointer(i).as_ptr() as usize);
            for index in 0..6 {
                let object = state.objects[index].lock().take();
                *state.objects[64 + index].lock() = object;
            }
            state.command.store(1, Ordering::Release);
            // Step 2: 分 3 轮与 CPU1 协同：验证异核释放落入 shared 链，首次再分配搬入 local 链，第二次分配直接命中 local。
            for round in 1..=3 {
                state.wait(&token, || {
                    state.acknowledgement.load(Ordering::Acquire) == round
                })?;
                let before = slub::cpu_snapshot(CLASS, 0);
                let shared = slub::slab_snapshot(anchor);
                assert_eq!(before.active, head);
                assert_eq!(before.local_count, 0);
                assert_eq!(shared.location, SlabLocation::Active(0));
                assert_eq!(shared.inuse, count - 2);
                assert_ne!(shared.shared, 0);
                let first = (round - 1) * 2;
                state.allocate_object(first);
                assert!(identities[first..first + 2]
                    .contains(&(state.pointer(first).as_ptr() as usize)));
                let after = slub::cpu_snapshot(CLASS, 0);
                assert_eq!(after.active, head);
                assert_eq!(after.local_count, 1);
                let slab = slub::slab_snapshot(anchor);
                assert_eq!(slab.shared, 0);
                assert_eq!(slab.inuse, count);
                if round < 3 {
                    state.command.store(round + 1, Ordering::Release);
                }
                state.allocate_object(first + 1);
                assert!(identities[first..first + 2]
                    .contains(&(state.pointer(first + 1).as_ptr() as usize)));
                assert_ne!(state.pointer(first), state.pointer(first + 1));
                assert_eq!(slub::cpu_snapshot(CLASS, 0).local_count, 0);
            }
            assert!(state.objects[64..70]
                .iter()
                .all(|slot| slot.lock().is_none()));
            for index in 0..count {
                state.release_object(index);
            }
            state.done[0].store(1, Ordering::Release);
            state.park(&token)
        })?;

        f.until(|| {
            f.state.done[0].load(Ordering::Acquire) == 1
                && f.state.done[1].load(Ordering::Acquire) == 1
        })
    }
}

/// 用于观测 `Arc` 跨核引用计数归零与最终析构所在物理 CPU 的测试载荷。
struct Payload {
    /// 用于校验存活期间堆内存内容完整性的固定特征字数组。
    words: [usize; 12],
    /// 指向多核共享控制块的弱引用，供 `Drop` 时安全上报析构计数与所在 CPU 编号。
    state: alloc::sync::Weak<State>,
}
impl Payload {
    /// 校验载荷数组中的 12 个特征字未被篡改。
    fn check(&self) {
        for (i, &word) in self.words.iter().enumerate() {
            assert_eq!(word, i * 37 + 11);
        }
    }
}
impl Drop for Payload {
    /// 在最后一个 `Arc<Payload>` 强引用被释放（Drop）时，将当前运行 CPU 编号写入 `drop_cpu` 并递增 `drops` 计数。
    fn drop(&mut self) {
        if let Some(state) = self.state.upgrade() {
            state.drop_cpu.store(cpu::current().id, Ordering::Relaxed);
            state.drops.fetch_add(1, Ordering::Release);
        }
    }
}

/// 内核堆引用计数智能指针（`Arc`）本核与跨核最终强引用释放（Drop）测试套件。
#[ktest::suite(fixture = SmpFixture, tags("mm", "heap", "arc"), timeout_ms = 30000)]
mod arc_ownership {
    use super::*;
    /// 验证在创建核（CPU0）本地释放最后一个 `Arc` 强引用时，析构函数精确执行一次且记录为 CPU0。
    ///
    /// - **测试目标**：确认 `Arc` 克隆、先释放副本再释放末尾强引用的单核基准生命周期行为正确。
    /// - **场景步骤**：调用 `final_drop(f, 0)` 在 CPU0 创建 `Arc<Payload>`，经一次克隆与释放验证尚未触发析构后，在 CPU0 释放最后一个强引用。
    /// - **预期结果**：`Payload::drop` 恰好执行 1 次，`drop_cpu` 为 `0`，堆内存按基线完整回收。
    #[test]
    fn local_final_drop(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        final_drop(f, 0)
    }
    /// 验证在 CPU0 创建 `Arc` 后将最后一个强引用移交 CPU1 独立任务释放时，析构与底层堆回收均在 CPU1 上正确完成。
    ///
    /// - **测试目标**：确认 `Arc<T>` 跨核传递所有权后，引用计数原子递减及最后一个强引用触发的跨核堆释放安全可靠。
    /// - **场景步骤**：调用 `final_drop(f, 1)`，由 CPU0 任务创建并克隆/缩减 `Arc<Payload>` 至唯一强引用后存入共享槽，发令唤醒 CPU1 任务校验数据并释放该最后一份 `Arc`。
    /// - **预期结果**：CPU0 释放克隆副本时不触发析构；CPU1 释放最后一个强引用后 `drops == 1` 且 `drop_cpu == 1`，夹具退出时六域账本闭合。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_final_drop(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        final_drop(f, 1)
    }
    /// 执行 `Arc<Payload>` 创建、引用计数增减及在指定目标核心 `target` 的绑定任务上释放最后一个强引用的通用流程。
    fn final_drop(f: &mut SmpFixture, target: usize) -> TestResult {
        // Step 1: 若目标释放核为远端核心（target != 0），预先在该核派生任务等待接管并释放最后一个强引用。
        if target != 0 {
            f.spawn(target, move |state, token| {
                state.wait(&token, || state.command.load(Ordering::Acquire) == 1)?;
                let mut slot = state.payload.lock();
                let payload = slot.as_ref().unwrap();
                assert_eq!(Arc::strong_count(payload), 1);
                payload.check();
                let payload = slot.take();
                drop(slot);
                drop(payload);
                state.done[target].store(1, Ordering::Release);
                state.park(&token)
            })?;
        }
        // Step 2: 在 CPU0 创建 Arc<Payload>，验证克隆副本释放不会提前析构，随后在本核释放或通知远端核释放。
        f.spawn(0, move |state, token| {
            let payload = Arc::try_new(Payload {
                words: core::array::from_fn(|i| i * 37 + 11),
                state: Arc::downgrade(&state),
            })
            .map_err(|_| TestError("NoMemory: Arc payload"))?;
            *state.payload.lock() = Some(payload);
            let mut slot = state.payload.lock();
            slot.as_ref().unwrap().check();
            let first = slot.take().unwrap();
            *slot = Some(Arc::clone(&first));
            drop(first);
            assert_eq!(state.drops.load(Ordering::Acquire), 0);
            assert_eq!(Arc::strong_count(slot.as_ref().unwrap()), 1);
            drop(slot);
            if target == 0 {
                state.payload.lock().as_ref().unwrap().check();
                let payload = state.payload.lock().take();
                drop(payload);
                state.done[0].store(1, Ordering::Release);
            } else {
                state.command.store(1, Ordering::Release);
            }
            state.park(&token)
        })?;
        // Step 3: 等待目标核心完成释放，核验析构总次数恰好为 1 且析构执行核心等于 target。
        f.until(|| f.state.done[target].load(Ordering::Acquire) == 1)?;
        assert_eq!(f.state.drops.load(Ordering::Acquire), 1);
        assert_eq!(f.state.drop_cpu.load(Ordering::Relaxed), target);
        assert!(f.state.payload.lock().is_none());
        Ok(())
    }
}
