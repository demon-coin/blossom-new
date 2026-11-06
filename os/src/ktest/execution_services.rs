//! 时间、软件定时器、工作队列、内核日志与追踪服务的测试基础设施及对象生命周期账务。
//!
//! # 模块职责与架构定位
//! 本模块作为 `ktest::execution_services` 测试组的公共支撑层，组织 `time`、`timer`、`workqueue`、`klog`、
//! `mixed` 与 `trace` 六个子模块，并向本组及 `execution_core` 测试套件提供统一的资源收敛基线与异步对象托管容器：
//! - 上游入口：由 `ktest::registry` 通过 [`anchors`] 收集全部 8 个测试套件描述符，并在套件前后运行 [`SuiteBaseline`]。
//! - 下游依赖：`crate::timer`（[`Timer`]）、`crate::workqueue`（[`Work`]、[`DelayedWork`]、[`WorkQueue`]）、
//!   `crate::klog`、`crate::fs::devfs` 以及 `crate::mm`（内存统计与缓存收缩）。
//!
//! # 核心设计与异步对象回收协议
//! ```text
//!   SuiteBaseline::set_up (套件级初始快照：MemoryStats + Workqueue MetadataStats)
//!        |
//!        v
//!   用例 set_up  ---> Nodes::new 预留固定容量；构造 Timer / DelayedWork / Work / WorkQueue 并保存在捕获边界外
//!        |
//!        v
//!   用例 Body    ---> 生产路径触发硬中断回调、底半部服务或工作线程池执行（可选经 on_zero 固定在 CPU 0 协调）
//!        |
//!        v
//!   Nodes::stop  ---> 严格按依赖逆序同步关闭：Timer -> DelayedWork -> Work -> WorkQueue (含救援线程 join)
//!        |
//!        v
//!   Nodes::tails ---> 等待异步回调彻底退出并归还锁外强引用（校验 Arc::strong_count 降至固定夹具持有的 3 份）
//!        |
//!        v
//!   reclaimed / SuiteBaseline::tear_down ---> 排空后台服务与每核页缓存，逐项比对内存与元数据完全归零收敛
//! ```
//!
//! # 锁获取顺序与并发契约
//! - [`Nodes`] 内部的四个注册表向量各自受独立的 [`SpinLock`] 保护，仅在推入新对象或按索引克隆强引用时短暂加锁。
//! - **严禁在持有 `Nodes` 内部自旋锁期间调用 `close_sync()` 或 `QueueOwner::stop()`**：`close_sync` 可能阻塞睡眠等待正在运行的回调退出，
//!   且回调本身可能访问测试状态；必须先在锁内按索引克隆出 `Pin<Arc<T>>` 或 `Arc<QueueOwner>`，释放自旋锁后再执行同步关闭。

mod klog;
mod mixed;
mod time;
mod timer;
mod trace;
mod workqueue;

use crate::sync::SpinLock;
use crate::timer::{Timer, TimerKind};
use crate::workqueue::{DelayedWork, Work, WorkQueue, WorkQueueConfig};
use crate::{arch, cpu, mm, sched};
use ::ktest::{TestError, TestResult};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::ops::Deref;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};

/// 内存分配器六项指标与工作队列存活元数据计数的联合资源基线快照。
pub(crate) type Baseline = (mm::MemoryStats, crate::workqueue::testing::MetadataStats);

/// 跨整个测试套件生命周期校验全局内存与工作队列元数据无泄漏的套件级夹具（Suite Resource）。
pub(crate) struct SuiteBaseline(Baseline);

impl ::ktest::Fixture for SuiteBaseline {
    /// 在套件首个用例开始前等待后台常驻服务静止，记录并输出初始内存与工作队列元数据基线。
    fn set_up(context: &mut ::ktest::TestContext<'_>, _: &()) -> Result<Self, ::ktest::TestError> {
        let initial = baseline();
        context.diagnostic(format_args!("EXECUTION SUITE initial={:?}", initial));
        Ok(Self(initial))
    }

    /// 在套件全部用例及其夹具释放完毕后重新采集基线，校验所有动态分配页、SLUB 对象与工作队列元数据完全复原。
    fn tear_down(self, context: &mut ::ktest::TestContext<'_>) -> TestResult {
        let actual = baseline();
        context.diagnostic(format_args!(
            "EXECUTION SUITE initial={:?} final={:?}",
            self.0, actual
        ));
        if actual == self.0 {
            Ok(())
        } else {
            Err(TestError("execution suite resources did not converge"))
        }
    }
}

/// 基于底层硬件单调计数器（15 秒超时上限）轮询等待条件成立，未满足时通过 `sched::yield_now()` 主动让出 CPU。
pub(crate) fn wait(mut predicate: impl FnMut() -> bool, name: &'static str) -> TestResult {
    let end = arch::read_counter()
        .checked_add(arch::counter_frequency().checked_mul(15).unwrap())
        .unwrap();
    while !predicate() {
        if end <= arch::read_counter() {
            return Err(TestError(name));
        }
        sched::yield_now();
    }
    Ok(())
}

/// 绑定在 CPU 0 固定协调线程栈上的异常恢复守卫，确保协调动作中途返回错误或发生子线程失败时自动放行阻塞门控。
struct Coordinator<T> {
    /// 被协调用例共享状态的强引用。
    owner: Arc<T>,
    /// 协调失败或异常展开时用于打开业务阻塞门控的释放回调。
    release: fn(&T),
    /// 标记协调动作是否正常完成且期间无任何子线程报错；为 `false` 时在析构中执行 `release`。
    success: bool,
}

impl<T> Drop for Coordinator<T> {
    /// 若协调动作未能成功完成，调用 `release` 回调解除其他工作线程的等待状态以便父线程进入清理流程。
    fn drop(&mut self) {
        if !self.success {
            (self.release)(&self.owner);
        }
    }
}

/// 在 CPU 0 上派生固定亲和性的协调子线程执行 `action` 并同步等待其退出；若中途失败则由 [`Coordinator`] 自动释放门控。
pub(super) fn on_zero<T: Send + Sync + 'static>(
    owner: Arc<T>,
    release: fn(&T),
    action: impl FnOnce() -> TestResult + Send + 'static,
) -> TestResult {
    // Step 1: 在 CPU 0 上启动携带异常门控释放守卫 `Coordinator` 的子线程执行协调逻辑。
    let child = crate::ktest::spawn_result_on(0, move |_| {
        let mut guard = Coordinator {
            owner,
            release,
            success: false,
        };
        let result = action();
        guard.success = result.is_ok() && !crate::ktest::child_failed();
        result.map(|()| 0)
    })?;

    // Step 2: 阻塞等待协调子线程彻底退出内核栈，并核对子线程状态与退出码。
    let value = child.join()?;
    if crate::ktest::child_failed() {
        return Err(TestError("fixed coordinator child failed"));
    }
    if value != 0 {
        return Err(TestError("fixed coordinator exit invariant"));
    }
    Ok(())
}

/// 等待日志、工作队列、软件定时器后台服务全部进入阻塞态且所有远端 CPU 进入 idle 后，收缩并排空内存缓存，采集当前资源快照。
pub(crate) fn baseline() -> Baseline {
    // Step 1: 等待内核常驻后台线程处理完挂起任务并阻塞，同时确认除当前核外的所有在线 CPU 均已进入 idle 循环。
    crate::klog::testing::wait_service_blocked();
    crate::workqueue::testing::wait_services_blocked();
    timer::wait_services_blocked();
    wait(
        || {
            (0..cpu::count())
                .all(|id| id == cpu::current().id || cpu::get(id).idle.load(Ordering::Acquire))
        },
        "remote scheduler resource tail",
    )
    .unwrap();

    // Step 2: 收缩 SLUB 空闲 slab 并排空每核 PCP 页缓存，消除延迟释放与每核缓存抖动后读取统计快照。
    mm::shrink_all();
    mm::drain_all_pages();
    (mm::stats(), crate::workqueue::testing::metadata_stats())
}

/// 重新采集全局静止基线并与预期快照 `expected` 逐项比对，确认临时物理内存与工作队列元数据均已完全回收。
pub(crate) fn reclaimed(expected: Baseline) -> TestResult {
    let actual = baseline();
    if actual.0 != expected.0 {
        return Err(TestError("temporary execution memory did not converge"));
    }
    if actual.1 != expected.1 {
        return Err(TestError("temporary execution metadata did not converge"));
    }
    Ok(())
}

/// 挂载一个独立的只读内存 `devfs` 实例并按给定路径名与标志打开内核观测设备文件（如日志或追踪设备）。
pub(super) fn open_observation(
    name: &[u8],
    flags: u32,
) -> Result<crate::fs::FileRef, crate::fs::FsError> {
    use crate::block::AccessMode;
    use crate::fs::{self, Filesystem, FsContext, InodeCache, ResolveMode, Vfs};

    let fs = fs::devfs::try_new(0o755)?;
    let cache = Arc::try_new(InodeCache::try_new(16)?).map_err(|_| fs::FsError::NoMemory)?;
    let vfs = Vfs::try_new(Filesystem::Memory(fs), cache, 16, 1, AccessMode::ReadOnly)?;
    let context = FsContext::new(vfs.root(), vfs.root())?;
    let path = vfs.resolve(&context, name, ResolveMode::Follow)?;
    fs::open_path_with_flags(&vfs, path, flags, fs::FileIoContext::Kernel)
}

/// 工作队列（[`WorkQueue`]）的单次同步关闭包装器，供夹具与测试代码通过 `Arc` 共享引用并保证 `close_sync` 幂等且不并发重入。
pub(super) struct QueueOwner {
    /// 底层工作队列实例；在 `Nodes::queue` 完成两阶段构造后恒为 `Some`。
    queue: Option<WorkQueue>,
    /// 标记底层 `WorkQueue::close_sync`（含救援线程回收）是否已彻底执行完毕返回。
    closed: AtomicBool,
    /// 标记是否已有调用方正在执行关闭流程，用于拦截未收敛的并发重复关闭。
    closing: AtomicBool,
}

impl QueueOwner {
    /// 同步关闭底层工作队列并排空在途工作项与救援线程；若此前已完成关闭则直接返回。
    pub(super) fn close_sync(&self) {
        self.stop().unwrap();
    }

    /// 执行一次性的同步关闭状态转换：仅在 `WorkQueue::close_sync` 完整返回后才置位 `closed` 标志。
    fn stop(&self) -> TestResult {
        if self.closed.load(Ordering::Acquire) {
            return Ok(());
        }
        if self.closing.swap(true, Ordering::AcqRel) {
            return Err(TestError("work queue close did not converge"));
        }

        self.deref().close_sync();
        self.closed.store(true, Ordering::Release);
        Ok(())
    }
}

impl Deref for QueueOwner {
    type Target = WorkQueue;

    /// 解引用获取已构造完成的底层 [`WorkQueue`] 引用。
    fn deref(&self) -> &Self::Target {
        self.queue.as_ref().expect("unpublished work queue")
    }
}

/// 托管单个测试用例在捕获边界外持有的全部定时器、普通工作项、延迟工作项与工作队列实例的注册容器。
pub(super) struct Nodes {
    /// 当前用例创建的软件定时器强引用列表（容量在 `Nodes::new` 中预留，由自旋锁保护）。
    timers: SpinLock<Vec<Pin<Arc<Timer>>>>,
    /// 当前用例创建的普通工作队列节点强引用列表（由自旋锁保护）。
    works: SpinLock<Vec<Pin<Arc<Work>>>>,
    /// 当前用例创建的延迟工作节点列表及其是否已完成 `close_sync` 的布尔标记（由自旋锁保护）。
    delayed: SpinLock<Vec<(Pin<Arc<DelayedWork>>, bool)>>,
    /// 当前用例创建的工作队列域包装器列表（由自旋锁保护）。
    queues: SpinLock<Vec<Arc<QueueOwner>>>,
}

impl Nodes {
    /// 按给定上限一次性预分配四类异步对象的登记槽位，避免在后续测试运行或内存受限阶段触发向量扩容。
    pub(super) fn new(
        timer_capacity: usize,
        work_capacity: usize,
        delayed_capacity: usize,
        queue_capacity: usize,
    ) -> Result<Self, TestError> {
        let mut timers = Vec::new();
        let mut works = Vec::new();
        let mut delayed = Vec::new();
        let mut queues = Vec::new();
        timers
            .try_reserve_exact(timer_capacity)
            .map_err(|_| TestError("NoMemory: timer owners"))?;
        works
            .try_reserve_exact(work_capacity)
            .map_err(|_| TestError("NoMemory: work owners"))?;
        delayed
            .try_reserve_exact(delayed_capacity)
            .map_err(|_| TestError("NoMemory: delayed owners"))?;
        queues
            .try_reserve_exact(queue_capacity)
            .map_err(|_| TestError("NoMemory: queue owners"))?;
        Ok(Self {
            timers: SpinLock::new(timers),
            works: SpinLock::new(works),
            delayed: SpinLock::new(delayed),
            queues: SpinLock::new(queues),
        })
    }

    /// 创建绑定到指定逻辑 CPU 的软件定时器，在首次启动（arm）前将其强引用登记到夹具容器并返回。
    pub(super) fn timer(
        &self,
        cpu: usize,
        kind: TimerKind,
        callback: fn(Pin<&Timer>),
        context: usize,
    ) -> Pin<Arc<Timer>> {
        let owner = Timer::try_new(cpu, kind, callback, context).expect("timer allocation");
        let mut slots = self.timers.lock();
        assert!(slots.len() < slots.capacity());
        slots.push(owner.clone());
        owner
    }

    /// 先分配 `QueueOwner` 壳再构造底层 [`WorkQueue`]，成功后登记到夹具容器并返回共享句柄。
    pub(super) fn queue(&self, config: WorkQueueConfig) -> Arc<QueueOwner> {
        {
            let slots = self.queues.lock();
            assert!(slots.len() < slots.capacity());
        }
        let mut owner = Arc::try_new(QueueOwner {
            queue: None,
            closed: AtomicBool::new(false),
            closing: AtomicBool::new(false),
        })
        .expect("queue owner allocation");
        Arc::get_mut(&mut owner).unwrap().queue =
            Some(WorkQueue::try_new(config).expect("work queue allocation"));
        let mut slots = self.queues.lock();
        assert!(slots.len() < slots.capacity());
        slots.push(owner.clone());
        owner
    }

    /// 在指定工作队列上创建普通 [`Work`] 节点，在入队前将其强引用登记到夹具容器并返回。
    pub(super) fn work(
        &self,
        queue: &WorkQueue,
        cpu: usize,
        callback: fn(Pin<&Work>),
        context: usize,
    ) -> Pin<Arc<Work>> {
        let owner = Work::try_new(queue, cpu, callback, context).expect("work allocation");
        let mut slots = self.works.lock();
        assert!(slots.len() < slots.capacity());
        slots.push(owner.clone());
        owner
    }

    /// 在指定工作队列上创建 [`DelayedWork`] 延迟工作节点，在提交延迟调度前登记到夹具容器并返回。
    pub(super) fn delayed(
        &self,
        queue: &WorkQueue,
        cpu: usize,
        callback: fn(Pin<&DelayedWork>),
        context: usize,
    ) -> Pin<Arc<DelayedWork>> {
        let owner =
            DelayedWork::try_new(queue, cpu, callback, context).expect("delayed allocation");
        let mut slots = self.delayed.lock();
        assert!(slots.len() < slots.capacity());
        slots.push((owner.clone(), false));
        owner
    }

    /// 对夹具内登记的指定 [`DelayedWork`] 执行同步关闭（停止内部定时器与工作项）并标记为已关闭，重复调用时自动跳过。
    pub(super) fn close_delayed(&self, object: Pin<&DelayedWork>) {
        let (index, complete) = {
            let slots = self.delayed.lock();
            let index = slots
                .iter()
                .position(|(owner, _)| core::ptr::eq(owner.as_ref().get_ref(), object.get_ref()))
                .expect("delayed owner outside fixture");
            (index, slots[index].1)
        };
        if !complete {
            object.close_sync();
            self.delayed.lock()[index].1 = true;
        }
    }

    /// 按“定时器 -> 延迟工作项 -> 普通工作项 -> 工作队列域”的依赖逆序同步停止夹具中登记的全部异步对象。
    pub(super) fn stop(&self) -> TestResult {
        // Step 1: 停止全部独立软件定时器，等待正在执行的硬中断或底半部定时回调退出。
        let timers = self.timers.lock().len();
        for index in 0..timers {
            let owner = self.timers.lock()[index].clone();
            let state = crate::timer::testing::snapshot(owner.as_ref());
            if !state.closed || state.running || state.queued {
                owner.as_ref().close_sync();
            }
        }

        // Step 2: 停止全部延迟工作项，确保其内部定时器不再向后续将要关闭的工作队列提交新任务。
        let delayed = self.delayed.lock().len();
        for index in 0..delayed {
            let owner = self.delayed.lock()[index].0.clone();
            self.close_delayed(owner.as_ref());
        }

        // Step 3: 停止全部普通工作项，排空或取消队列中挂起与运行中的回调。
        let works = self.works.lock().len();
        for index in 0..works {
            let owner = self.works.lock()[index].clone();
            let state = crate::workqueue::testing::snapshot(owner.as_ref());
            if !state.closed || state.running || state.queued {
                owner.as_ref().close_sync();
            }
        }

        // Step 4: 最后同步关闭所有工作队列域并回收其救援线程。
        let queues = self.queues.lock().len();
        for index in 0..queues {
            let owner = self.queues.lock()[index].clone();
            owner.stop()?;
        }
        Ok(())
    }

    /// 等待夹具中所有定时器、普通工作项与延迟工作项由异步执行上下文持有的临时强引用全部在锁外释放归还。
    pub(super) fn tails(&self) -> TestResult {
        let timers = self.timers.lock().len();
        for index in 0..timers {
            let object = self.timers.lock()[index].clone();
            owned_tail(&object)?;
        }

        let works = self.works.lock().len();
        for index in 0..works {
            let object = self.works.lock()[index].clone();
            owned_tail(&object)?;
        }

        let delayed = self.delayed.lock().len();
        for index in 0..delayed {
            let object = self.delayed.lock()[index].0.clone();
            owned_tail(&object)?;
        }
        Ok(())
    }
}

/// 等待固定在堆上的异步节点归还所有队列与回调执行期强引用，仅保留夹具容器、`tails` 局部变量与本次观察克隆共 3 份强引用。
fn owned_tail<T>(object: &Pin<Arc<T>>) -> TestResult {
    // SAFETY:
    // 1. 此处拆包的是 `object.clone()` 产生的临时 `Pin<Arc<T>>` 副本，仅用于传入 `Arc::strong_count` 读取控制块引用计数；
    // 2. 绝不获取可变引用或移出底层 `T` 实例，且原始 `object` 在整个等待期间保持 `Pin` 固定存活。
    let observed = unsafe { Pin::into_inner_unchecked(object.clone()) };
    wait(
        || Arc::strong_count(&observed) == 3,
        "asynchronous ownership did not return",
    )
}

/// 返回 `execution_services` 测试组下全部 8 个子套件由 `#[ktest::suite]` 宏生成的静态注册槽引用。
pub(crate) fn anchors() -> [&'static ::ktest::SuiteSlot; 8] {
    [
        time::cases::__ktest_anchor(),
        timer::cases::__ktest_anchor(),
        workqueue::cases::__ktest_anchor(),
        klog::records::__ktest_anchor(),
        klog::uart::__ktest_anchor(),
        klog::fatal::__ktest_anchor(),
        mixed::cases::__ktest_anchor(),
        trace::cases::__ktest_anchor(),
    ]
}
