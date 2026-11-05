//! 时间、定时器、工作域与日志的具名场景；成功fixture在主体捕获边界外承担原拥有责任。

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

pub(crate) type Baseline = (mm::MemoryStats, crate::workqueue::testing::MetadataStats);

/// 套件准备后到最后用例控制对象释放后，实际比较跨例的内存与任务资源账。
pub(crate) struct SuiteBaseline(Baseline);

impl ::ktest::Fixture for SuiteBaseline {
    /// 常驻服务停稳后保存实际六域和元数据基线，不持有或重置生产资源。
    fn set_up(context: &mut ::ktest::TestContext<'_>, _: &()) -> Result<Self, ::ktest::TestError> {
        let initial = baseline();
        context.diagnostic(format_args!("EXECUTION SUITE initial={:?}", initial));
        Ok(Self(initial))
    }

    /// 原suite作用域已join后检查全部case控制对象归还，保真实前后值用于定位。
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

/// 只沿原始硬件counter限制协调等待，生产进度仍经过真实IRQ、等待源与scheduler。
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

struct Coordinator<T> {
    owner: Arc<T>,
    release: fn(&T),
    success: bool,
}

impl<T> Drop for Coordinator<T> {
    /// 当前动作已退栈后只释放该具体fixture的测试gate；远核IRQ不能阻挡Any父恢复清理。
    fn drop(&mut self) {
        if !self.success {
            (self.release)(&self.owner);
        }
    }
}

/// 原scope发表真正fixed-0协调Task，闭包只捕获独立拥有Arc；suite仍是原可迁核Task。
pub(super) fn on_zero<T: Send + Sync + 'static>(
    owner: Arc<T>,
    release: fn(&T),
    action: impl FnOnce() -> TestResult + Send + 'static,
) -> TestResult {
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

    let value = child.join()?;
    if crate::ktest::child_failed() {
        return Err(TestError("fixed coordinator child failed"));
    }
    if value != 0 {
        return Err(TestError("fixed coordinator exit invariant"));
    }
    Ok(())
}

/// 常驻服务和远核离栈尾部结束以后排空MM缓存，不抵消SLUB占用。
pub(crate) fn baseline() -> Baseline {
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

    mm::shrink_all();
    mm::drain_all_pages();
    (mm::stats(), crate::workqueue::testing::metadata_stats())
}

/// fixture及原控制对象释放后同时核对完整六域和真实Task/Worker/Domain payload。
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

/// 用独立只读 devfs 的真实名字和 inode 打开观测设备，返回的 Path 保活原挂载。
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

/// 原WorkQueue只有一个关闭责任；Arc仅保活同一个域owner，不复制域或执行者。
pub(super) struct QueueOwner {
    queue: Option<WorkQueue>,
    closed: AtomicBool,
    closing: AtomicBool,
}

impl QueueOwner {
    /// 显式关闭一次，原实现仍负责排空颜色、释放预留和join真实rescuer。
    pub(super) fn close_sync(&self) {
        self.stop().unwrap();
    }

    /// 关闭尚未真实返回时保留原owner，拒绝把提前closed位当成rescuer已经join。
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

    /// 只有真实构造成功的owner才能进入正文或原flush。
    fn deref(&self) -> &Self::Target {
        self.queue.as_ref().expect("unpublished work queue")
    }
}

/// 只保存这些场景实际产生的三种节点与原域owner；没有动态类型、回滚或第二执行池。
pub(super) struct Nodes {
    timers: SpinLock<Vec<Pin<Arc<Timer>>>>,
    works: SpinLock<Vec<Pin<Arc<Work>>>>,
    delayed: SpinLock<Vec<(Pin<Arc<DelayedWork>>, bool)>>,
    queues: SpinLock<Vec<Arc<QueueOwner>>>,
}

impl Nodes {
    /// case setup预备登记容量，后续节点发表前无分配地保留原owner的强引用。
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

    /// 原Timer构造后、任何arm之前把同一固定分配保存在catch外。
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

    /// 先准备空owner控制块，再创建真实域；失败没有已发表Work或未关闭域遗失。
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

    /// 在queue接受任何票之前保活同一个真实Work。
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

    /// 在delay登记之前保存原DelayedWork，不复制内部timer/work票。
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

    /// 记录该D真实close_sync已经返回；domain.closed不能代替内部timer的尾部责任。
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

    /// 具体fixture已停止所有生产者、释放业务gate及测试停点以后停止真实节点与原域。
    pub(super) fn stop(&self) -> TestResult {
        let timers = self.timers.lock().len();
        for index in 0..timers {
            let owner = self.timers.lock()[index].clone();
            let state = crate::timer::testing::snapshot(owner.as_ref());
            if !state.closed || state.running || state.queued {
                owner.as_ref().close_sync();
            }
        }

        let delayed = self.delayed.lock().len();
        for index in 0..delayed {
            let owner = self.delayed.lock()[index].0.clone();
            self.close_delayed(owner.as_ref());
        }

        let works = self.works.lock().len();
        for index in 0..works {
            let owner = self.works.lock()[index].clone();
            let state = crate::workqueue::testing::snapshot(owner.as_ref());
            if !state.closed || state.running || state.queued {
                owner.as_ref().close_sync();
            }
        }

        let queues = self.queues.lock().len();
        for index in 0..queues {
            let owner = self.queues.lock()[index].clone();
            owner.stop()?;
        }
        Ok(())
    }

    /// 原scope已经join后确认每种异步票均归还；失败仍保留所有fixture拥有者供case-abort。
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

/// 仅夹具保留的原强引用允许留下，close清位不能代替锁外拥有票归还。
fn owned_tail<T>(object: &Pin<Arc<T>>) -> TestResult {
    // SAFETY: 克隆仅观察固定Arc控制块；原Pin不移出对象并保活到观察结束。
    let observed = unsafe { Pin::into_inner_unchecked(object.clone()) };
    wait(
        || Arc::strong_count(&observed) == 3,
        "asynchronous ownership did not return",
    )
}

/// 每个返回项都是宏生成的实际只读注册槽，不是手写case目录。
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
