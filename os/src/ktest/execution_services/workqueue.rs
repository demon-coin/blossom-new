//! 工作队列（WorkQueue）与延迟工作（DelayedWork）子系统内核自测模块。
//!
//! # 模块职责与架构定位
//! 本模块位于 `ktest::execution_services` 测试集中，负责验证工作队列（`crate::workqueue`）与
//! 延迟工作（`crate::workqueue::delayed`）的并发调度、工作线程（Worker）管理、OOM 应急救援
//! 以及生命周期回收契约。
//! - **上游调用者**：内核测试框架（`ktest`）及 `execution_services` 父模块的异常恢复测试。
//! - **下游依赖**：工作队列核心（`WorkQueue`、`Work`、`DelayedWork`）、高精度定时器服务
//!   （`crate::timer`）、任务与调度器（`crate::task`、`crate::sched`）以及物理页与 SLUB
//!   内存分配器（`crate::mm`）。
//!
//! # 核心设计与生命周期模型
//! 测试围绕固定地址的探测控制块 [`Probe`] 组织。每个 [`Probe`] 内嵌固定槽位数组 `entries`，
//! 通过自引用原始指针将槽位地址作为 `usize` 上下文传给 `Work` / `DelayedWork` 回调：
//!
//! ```text
//!   Scenario (CURRENT 全局夹具状态)
//!      │
//!      ├── nodes: Nodes (管理测试创建的 WorkQueue / Work / DelayedWork 强引用)
//!      ├── pressure / objects (OOM 压力测试暂存的物理页与 SLUB 堆块)
//!      └── probes: Vec<Arc<Probe>>
//!                      │
//!                      ▼
//!                 ┌───────────────────────────────────────────┐
//!                 │ Probe (Arc 固定堆分配)                     │
//!                 │  entries[i]: Entry { probe: *const Probe, │◄──┐
//!                 │                      index: i }           │   │
//!                 │  works[i]:   SpinLock<Option<Pin<Arc>>>   │   │ context(addr)
//!                 │  tasks[i]:   SpinLock<Option<Arc<Task>>>  │   │ 反向解析
//!                 │  gates[i]:   Completion (同步门闸)         │   │
//!                 └───────────────────┬───────────────────────┘   │
//!                                     │ &entries[i] as usize      │
//!                                     ▼                           │
//!                           Work / DelayedWork ───────────────────┘
//! ```
//!
//! ## 核心验证维度
//! 1. **重复入队与串行保证**：Pending 状态或睡眠执行期间重复 `queue()` 的合并语义，确保同一
//!    `Work` 绝不并发重入。
//! 2. **有序域（Ordered WorkQueue）與执行者关联（Worker Association）**：验证全域单并发、
//!    跨 CPU 提交归一化至 CPU0 FIFO 执行，以及重入排队任务绑定原 Worker、防止被跨域 `flush()`
//!    窃取执行者的死锁防护机制。
//! 3. **多级阻塞与动态 Worker 扩缩容**：工作回调链式睡眠时唤醒 Manager 创建动态 Worker，
//!    以及完工或取消后的自动退休回收。
//! 4. **内存耗尽（OOM）与 Rescuer 救援**：耗尽物理页与全部 SLUB 规格后，验证带 `reclaim`
//!    标志的队列启用预分配 Rescuer 线程推进内存回收，普通队列则进入冷却等待并在外部释放内存后恢复。
//! 5. **Flush 颜色边界与 DelayedWork 定时器移交**：验证 `flush()` 仅等待调用时刻已存在的批次，
//!    以及 `DelayedWork` 在定时器到期移交前后的同步取消与关闭引用计数归还。
//!
//! # 锁获取顺序（Lock Ordering）与并发契约
//! - 测试夹具内部自旋锁（`Scenario::probes`、`Scenario::owners`、`Scenario::pressure_counts`、
//!   `Probe::works[i]`、`Probe::tasks[i]`）均为叶子级短临界区锁。
//! - **严禁在持有上述测试自旋锁期间调用调度器接口（如 `sched::test_state`、`sched::yield_now`）
//!   或工作队列同步接口（如 `flush`、`cancel_sync`、`close_sync`）**；读取任务状态前必须先克隆
//!   `Arc<Task>` 并释放 `Probe::tasks[i]` 锁，防止与生产调度锁发生死锁。

use alloc::alloc::{alloc, dealloc};
use alloc::sync::Arc;
use core::alloc::Layout;
use core::hint::spin_loop;
use core::mem::MaybeUninit;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::sync::{Completion, SpinLock};
use crate::task::{self, State, Task};
use crate::time::Duration;
use crate::wait::Waiter;
use crate::workqueue::delayed::testing as delayed_testing;
use crate::workqueue::{
    testing, DelayedError, DelayedWork, QueueError, Work, WorkQueue, WorkQueueConfig,
};
use crate::{arch, cpu, mm, sched};

/// 单个 [`Probe`] 支持的最大并发工作槽位数。
const SLOTS: usize = 8;
/// 深度阻塞依赖链测试（`test_deep_blocking`）的链式阻塞层数。
const DEPTH: usize = 3;

/// OOM 压力测试预分配的物理页块与 SLUB 对象指针最大槽位容量。
const PRESSURE_CAPACITY: usize = 65536;
/// 传递给 `Work` 或 `DelayedWork` 回调的固定槽位定位项。
///
/// 存储在 [`Probe::entries`] 数组中，通过将其地址转为 `usize` 传入工作项的 `context` 字段，
/// 使回调函数能够同时获知所属的 [`Probe`] 实例地址与槽位下标。
struct Entry {
    /// 指向所属 [`Probe`] 实例的原始指针地址（构造阶段写入后只读）。
    probe: usize,
    /// 当前槽位在 [`Probe`] 各数组中的下标（`0..SLOTS`）。
    index: usize,
}
/// 工作队列测试用例的共享探测与同步控制块。
///
/// 集中维护单个测试场景中的工作项句柄、执行过回调的 Worker 任务句柄、同步门闸（`Completion`）
/// 以及各类原子统计计数器。
struct Probe {
    /// 固定地址的回调上下文入口数组，按槽位下标索引。
    entries: [Entry; SLOTS],
    /// 各槽位绑定的 `Work` 强引用，由各自的 `SpinLock` 保护，供回调内链式唤醒或自我重入使用。
    works: [SpinLock<Option<Pin<Arc<Work>>>>; SLOTS],
    /// 各槽位回调执行时记录的当前 `Task` 强引用，由各自的 `SpinLock` 保护，用于外部观测阻塞与退出状态。
    tasks: [SpinLock<Option<Arc<Task>>>; SLOTS],
    /// 各槽位的完成门闸，用于控制回调在指定阶段阻塞与放行。
    gates: [Completion; SLOTS],
    /// 各槽位通用的原子轮次或阶段计数器。
    counts: [AtomicUsize; SLOTS],
    /// 当前正在执行回调函数体的活跃并发数。
    active: AtomicUsize,
    /// 观测到的历史最大活跃并发数（用于验证单并发不重入契约）。
    maximum: AtomicUsize,
    /// 业务层累计提交的工作批次或事件计数。
    submitted: AtomicUsize,
    /// 回调函数累计消费的工作批次或完成计数。
    consumed: AtomicUsize,
    /// 回调函数被实际调用的总次数。
    callbacks: AtomicUsize,
    /// 通知阻塞中的回调或工作线程放行退出的原子标志。
    release: AtomicBool,
    /// 业务生产源开关：为 `true` 时允许回调继续自我重排（Self-requeue），为 `false` 时停止生产。
    source: AtomicBool,
    /// 标记辅助协调线程或刷新线程已启动并进入目标阶段。
    started: AtomicBool,
    /// 标记目标回调或辅助线程已执行完毕。
    done: AtomicBool,
    /// OOM 压力测试中暂存的物理页块数量。
    pages: AtomicUsize,
    /// OOM 压力测试中暂存的 SLUB 堆对象数量。
    objects: AtomicUsize,
    /// 由应急救援线程（Rescuer Worker）执行的回调次数。
    rescuer: AtomicUsize,
    /// 由普通工作线程（Ordinary Worker）执行的回调次数。
    ordinary: AtomicUsize,
    /// [`SourceOwner::close`] 显式停止操作被调用的次数。
    stops: AtomicUsize,
}

impl Probe {
    /// 分配并初始化 [`Probe`] 控制块，建立内部 `entries` 到自身的指针并注册到当前测试场景。
    fn new() -> Arc<Self> {
        let raw = Arc::into_raw(Arc::new(Self {
            entries: core::array::from_fn(|index| Entry { probe: 0, index }),
            works: [const { SpinLock::new(None) }; SLOTS],
            tasks: [const { SpinLock::new(None) }; SLOTS],
            gates: [const { Completion::new() }; SLOTS],
            counts: [const { AtomicUsize::new(0) }; SLOTS],
            active: AtomicUsize::new(0),
            maximum: AtomicUsize::new(0),
            submitted: AtomicUsize::new(0),
            consumed: AtomicUsize::new(0),
            callbacks: AtomicUsize::new(0),
            release: AtomicBool::new(false),
            source: AtomicBool::new(true),
            started: AtomicBool::new(false),
            done: AtomicBool::new(false),
            pages: AtomicUsize::new(0),
            objects: AtomicUsize::new(0),
            rescuer: AtomicUsize::new(0),
            ordinary: AtomicUsize::new(0),
            stops: AtomicUsize::new(0),
        }));
        for index in 0..SLOTS {
            // SAFETY: `raw` 来自上方唯一的 `Arc::into_raw`，此时尚未向任何线程或队列暴露共享引用；
            // `index < SLOTS` 保证指针偏移在 `entries` 数组边界内，写入后 `probe` 字段终生只读。
            unsafe {
                let entry = core::ptr::addr_of_mut!((*raw.cast_mut()).entries)
                    .cast::<Entry>()
                    .add(index);
                core::ptr::addr_of_mut!((*entry).probe).write(raw as usize);
            }
        }
        // 将初始化完毕的原始指针重新封装为 `Arc<Probe>` 并登记到全局场景列表中。
        {
            // SAFETY: `raw` 由上方 `Arc::into_raw` 产生且中途未构造其他 `Arc` 或释放内存，引用计数与所有权完全匹配。
            let owner = unsafe { Arc::from_raw(raw) };
            scenario().probes.lock().push(owner.clone());
            owner
        }
    }

    /// 在指定工作队列上创建绑定到槽位 `index` 的 `Work`，并将其强引用存入对应槽位。
    fn work(
        &self,
        queue: &WorkQueue,
        index: usize,
        owner: usize,
        callback: fn(Pin<&Work>),
    ) -> Pin<Arc<Work>> {
        let work = new_work(
            queue,
            owner,
            callback,
            &self.entries[index] as *const Entry as usize,
        );
        assert!(self.works[index].lock().replace(work.clone()).is_none());
        work
    }

    /// 获取槽位 `index` 中预先准备好的 `Work` 强引用克隆（执行过程不触发新的工作项分配）。
    fn next_work(&self, index: usize) -> Pin<Arc<Work>> {
        self.works[index]
            .lock()
            .as_ref()
            .expect("prepared work")
            .clone()
    }

    /// 将当前正在执行回调的内核任务（`Task`）记录到槽位 `index` 中。
    fn publish_task(&self, index: usize) {
        let current = task::current_task();
        assert!(self.tasks[index].lock().replace(current).is_none());
    }

    /// 检查槽位 `index` 记录的内核任务当前是否处于调度器的 `Blocked`（睡眠阻塞）状态。
    fn blocked(&self, index: usize) -> bool {
        let observed = self.tasks[index].lock().as_ref().map(Arc::clone);
        observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 清空所有槽位中保存的 `Work` 与 `Task` 强引用，在自旋锁临界区外释放引用计数。
    fn clear(&self) {
        for index in 0..SLOTS {
            let work = self.works[index].lock().take();
            let task = self.tasks[index].lock().take();
            drop((work, task));
        }
    }
}

impl Drop for Probe {
    /// 校验所有活跃回调已结束且槽位强引用已清空，并累加场景级的 [`Probe`] 析构计数。
    fn drop(&mut self) {
        assert_eq!(self.active.load(Ordering::Acquire), 0);
        assert!(self.works.iter().all(|work| work.lock().is_none()));
        assert!(self.tasks.iter().all(|task| task.lock().is_none()));
        scenario().operation_drops.fetch_add(1, Ordering::AcqRel);
    }
}

/// 从固定槽位 [`Entry`] 的内存地址还原共享的 [`Probe`] 引用与槽位下标。
///
/// # Safety
/// 1. `address` 必须指向一个存活的 `Arc<Probe>` 内部的 [`Entry`] 元素。
/// 2. 调用方必须保证该 `Arc<Probe>` 在返回的借用 `'a` 生命周期内始终存活，直到所有使用该 [`Entry`]
///    的工作项（`Work` / `DelayedWork`）和子任务均已同步关闭并归还异步引用计数。
/// 3. [`Entry`] 在 [`Probe::new`] 初始化完成后严禁发生任何可变修改，且返回的引用不得逃逸出当前回调执行上下文。
unsafe fn context<'a>(address: usize) -> (&'a Probe, usize) {
    // SAFETY: 调用方契约保证 `address` 是有效、对齐且在 `'a` 期间只读存活的 `Entry` 指针。
    let entry = unsafe { &*(address as *const Entry) };
    // SAFETY: `entry.probe` 在 `Probe::new` 中写入了所属 `Probe` 的堆分配首地址，且所属 `Arc<Probe>` 在 `'a` 期间保活且无独占可变借用。
    (unsafe { &*(entry.probe as *const Probe) }, entry.index)
}

/// 基于硬件计数器计算当前时刻起 10 秒后的绝对超时时间戳（Tick 数）。
fn raw_deadline() -> u64 {
    arch::read_counter()
        .checked_add(arch::counter_frequency().checked_mul(10).unwrap())
        .unwrap()
}

/// 循环检查条件 `predicate` 直到成立，未成立时主动让出 CPU（`yield_now`），超时则触发断言失败。
fn until(mut predicate: impl FnMut() -> bool, name: &str) {
    let end = raw_deadline();
    while !predicate() {
        assert!(
            arch::read_counter() < end,
            "work execution timeout: {}",
            name
        );
        sched::yield_now();
    }
}

/// 自旋忙等条件 `predicate` 直到成立（用于等待不能睡眠的远核下半部或原子停点握手），超时则触发断言失败。
fn spin_until(mut predicate: impl FnMut() -> bool, name: &str) {
    let end = raw_deadline();
    while !predicate() {
        assert!(arch::read_counter() < end, "work pause timeout: {}", name);
        spin_loop();
    }
}

/// 等待工作队列常驻 Worker、Manager 线程以及定时器服务线程全部进入空闲阻塞状态，且动态 Worker 完全退出。
pub(super) fn wait_services_blocked() {
    testing::wait_services_blocked();
    super::timer::wait_services_blocked();
}

/// 等待指定 `Work` 的所有异步执行引用归还，直到仅剩测试框架与调用方持有的 3 个基准强引用。
fn unique_work(work: &Pin<Arc<Work>>) {
    // SAFETY: 仅取出内部 `Arc<Work>` 用于读取引用计数，不会移动底层的 `Work` 实例。
    let observer = unsafe { Pin::into_inner_unchecked(work.clone()) };
    until(
        || Arc::strong_count(&observer) == 3,
        "work asynchronous reference returned",
    );
    drop(observer);
}

/// 在不调用取消或关闭操作的前提下，等待指定 `DelayedWork` 的异步强引用归还至 3 个基准强引用，并返回多余引用数。
fn unique_delayed(work: &Pin<Arc<DelayedWork>>) -> usize {
    // SAFETY: 仅取出内部 `Arc<DelayedWork>` 用于读取引用计数，不会移动底层的 `DelayedWork` 实例。
    let observer = unsafe { Pin::into_inner_unchecked(work.clone()) };
    until(
        || Arc::strong_count(&observer) == 3,
        "delayed asynchronous tickets returned",
    );
    let orphan = Arc::strong_count(&observer) - 3;
    assert_eq!(orphan, 0);
    orphan
}

/// 有序队列（Ordered WorkQueue）测试回调：验证任务统一归一化到 CPU0 执行、遵循严格 FIFO 顺序且并发度为 1。
fn ordered_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `ordered_case` 在所有工作项同步关闭、引用计数归还且提交线程 `join` 后才释放 `Probe`。
    let (probe, index) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, 0);
    assert_eq!(work.cpu(), 0);
    let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
    probe.maximum.fetch_max(active, Ordering::AcqRel);
    assert_eq!(active, 1);
    assert_eq!(probe.consumed.fetch_add(1, Ordering::AcqRel), index);
    sched::yield_now();
    assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// 测试跨 CPU 向有序队列（Ordered WorkQueue）提交任务时的 CPU0 归一化绑定与全域单并发 FIFO 保证。
fn ordered_case() -> (usize, usize) {
    let queue = new_queue(WorkQueueConfig {
        max_active: 4,
        ordered: true,
        reclaim: false,
    });
    let probe = Probe::new();
    let works: [Pin<Arc<Work>>; 4] = core::array::from_fn(|index| {
        probe.work(&queue, index, index % cpu::count(), ordered_callback)
    });
    // Step 1: 在最后一个 CPU 上生成提交线程，将 4 个工作项依次入队。
    let submissions = works.clone();
    let submit_cpu = cpu::count() - 1;
    let handle = crate::ktest::spawn_result_on(submit_cpu, move |_| {
        Ok({
            assert_eq!(cpu::current().id, submit_cpu);
            for work in &submissions {
                assert!(work.as_ref().queue().unwrap());
            }
            4
        })
    })
    .unwrap();
    // Step 2: 等待提交完成并刷新队列，验证 4 个工作项按序执行且最大并发度为 1。
    assert_eq!(handle.join().unwrap(), 4);
    queue.flush();
    assert_eq!(probe.consumed.load(Ordering::Acquire), 4);
    assert_eq!(probe.maximum.load(Ordering::Acquire), 1);

    // Step 3: 同步关闭所有工作项与队列，校验异步强引用全部归还。
    for work in &works {
        work.as_ref().close_sync();
    }
    queue.close_sync();
    probe.clear();
    for work in &works {
        unique_work(work);
    }
    drop((works, queue, probe));
    (1, 1)
}

/// 有序队列执行者关联（Worker Association）测试回调：
/// 槽位 0（有序域任务 A）在首轮入队槽位 1（无关域任务 B）并自我重排，次轮阻塞在门闸上且验证仍由原 Worker 执行；
/// 槽位 1（无关域任务 B）在另一 Worker 上执行并对任务 A 调用跨域 `flush()`。
fn ordered_association_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `ordered_association_case` 保持 `Probe` 存活，直到所有回调完成、工作项与队列同步关闭且引用归还。
    let (probe, index) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, 0);
    assert_eq!(work.cpu(), 0);
    assert!(!testing::is_rescuer());
    if index == 0 {
        let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
        probe.maximum.fetch_max(active, Ordering::AcqRel);
        assert_eq!(active, 1);
        let round = probe.counts[0].fetch_add(1, Ordering::AcqRel);
        assert_eq!(probe.consumed.fetch_add(1, Ordering::AcqRel), round);
        if round == 0 {
            probe.publish_task(0);
            if scenario().stop.load(Ordering::Acquire) {
                return;
            }
            assert!(probe.source.load(Ordering::Acquire));
            assert!(probe.next_work(1).as_ref().queue().unwrap());
            assert!(work.queue().unwrap());
        } else {
            assert_eq!(round, 1);
            let original = probe.tasks[0].lock().as_ref().unwrap().id();
            assert_eq!(task::current_task_id(), Some(original));
            probe.gates[0].wait();
        }
        assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
    } else {
        assert_eq!(index, 1);
        probe.publish_task(1);
        let original = probe.tasks[0].lock().as_ref().unwrap().id();
        assert_ne!(
            task::current_task_id(),
            Some(original),
            "ordered association handed its executor to cross-domain flush"
        );
        assert_eq!(probe.counts[1].fetch_add(1, Ordering::AcqRel), 0);
        assert!(probe.next_work(0).as_ref().flush());
        assert_eq!(probe.consumed.fetch_add(1, Ordering::AcqRel), 2);
        assert_eq!(probe.counts[1].fetch_add(1, Ordering::AcqRel), 1);
    }
}

/// 测试有序队列中的自我重排任务保留原 Worker 关联，且跨域 `flush()` 在独立 Worker 中阻塞等待而不会窃取原执行者。
fn ordered_association_case() {
    let ordered = new_queue(WorkQueueConfig {
        max_active: 4,
        ordered: true,
        reclaim: false,
    });
    let unrelated = new_queue(WorkQueueConfig {
        max_active: 1,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let a = probe.work(&ordered, 0, 0, ordered_association_callback);
    let b = probe.work(&unrelated, 1, 0, ordered_association_callback);
    // Step 1: 启动有序域任务 A，等待 A 的第二轮回调与无关域任务 B 的跨域 `flush()` 双双进入阻塞状态。
    assert!(a.as_ref().queue().unwrap());
    until(
        || probe.blocked(0) && probe.blocked(1),
        "ordered next round and cross-domain flush truly blocked",
    );
    assert_eq!(probe.counts[0].load(Ordering::Acquire), 2);
    assert_eq!(probe.counts[1].load(Ordering::Acquire), 1);
    assert_eq!(probe.consumed.load(Ordering::Acquire), 2);
    assert_eq!(probe.maximum.load(Ordering::Acquire), 1);
    let a_state = testing::snapshot(a.as_ref());
    let b_state = testing::snapshot(b.as_ref());
    assert!(a_state.running && !a_state.queued);
    assert!(b_state.running && !b_state.queued);
    assert_ne!(a_state.worker, b_state.worker);

    // Step 2: 关闭继续生产标志并放行门闸，验证任务 A 第二轮完成且任务 B 的跨域 `flush()` 正常返回。
    probe.source.store(false, Ordering::Release);
    probe.gates[0].complete();
    until(
        || probe.counts[1].load(Ordering::Acquire) == 2,
        "ordered next round completed and legal cross-domain flush returned",
    );
    ordered.flush();
    unrelated.flush();
    assert_eq!(probe.consumed.load(Ordering::Acquire), 3);
    assert_eq!(probe.active.load(Ordering::Acquire), 0);

    // Step 3: 同步关闭工作项与两个队列，验证异步引用全部归还。
    a.as_ref().close_sync();
    b.as_ref().close_sync();
    ordered.close_sync();
    unrelated.close_sync();
    probe.clear();
    unique_work(&a);
    unique_work(&b);
    drop((a, b, ordered, unrelated, probe));
}

/// 重复入队合并测试回调：验证业务批次计数可累积，首轮回调阻塞期间的新入队请求会合并为下一轮串行执行，绝不并发重入。
fn duplicate_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `test_duplicates` 持有 `Probe` 直到门闸放行、工作项同步关闭且异步强引用全部归还。
    let (probe, _) = unsafe { context(work.context()) };
    let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
    probe.maximum.fetch_max(active, Ordering::AcqRel);
    assert_eq!(active, 1);
    let round = probe.callbacks.fetch_add(1, Ordering::AcqRel);
    let submitted = probe.submitted.load(Ordering::Acquire);
    let consumed = probe.consumed.swap(submitted, Ordering::AcqRel);
    assert!(consumed < submitted);
    if round == 0 {
        probe.publish_task(0);
        probe.gates[0].wait();
    }
    assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// Worker 认领（Claim）清除 Pending 标志后的停点测试回调：验证运行期间再次入队会触发串行的第二轮回调。
fn started_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `test_duplicates` 在解除 `Started` 停点、同步关闭 `started` 工作项并确认引用归还后才释放 `Probe`。
    let (probe, index) = unsafe { context(work.context()) };
    let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
    assert_eq!(active, 1);
    probe.maximum.fetch_max(active, Ordering::AcqRel);
    probe.counts[index].fetch_add(1, Ordering::AcqRel);
    assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// 测试工作项在入队等待期（Pending）与回调睡眠运行期（Running）重复提交时的合并语义与单实例串行保证。
fn test_duplicates() {
    let queue = new_queue(WorkQueueConfig {
        max_active: 4,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let work = probe.work(&queue, 0, 0, duplicate_callback);
    let started = if 1 < cpu::count() {
        Some(probe.work(&queue, 1, 1, started_callback))
    } else {
        None
    };

    // Step 1: 在首次入队后连续重复提交 3 次，验证 Pending 状态下的重复提交被合并（返回 `false`）。
    probe.submitted.store(8, Ordering::Release);
    assert!(work.as_ref().queue().unwrap());
    let mut before_duplicates = 0;
    for _ in 0..3 {
        before_duplicates += usize::from(!work.as_ref().queue().unwrap());
    }
    assert_eq!(before_duplicates, 3);
    // Step 2: 等待首轮回调进入睡眠阻塞，此时再次入队应成功标记为 `queued`（与 `running` 并存），后续 3 次重复入队继续合并。
    until(|| probe.blocked(0), "first duplicate callback blocked");
    probe.submitted.fetch_add(8, Ordering::Release);
    assert!(work.as_ref().queue().unwrap());
    let state = testing::snapshot(work.as_ref());
    let overlap = usize::from(state.running && state.queued);
    assert_eq!(overlap, 1);
    {
        let observed = Arc::clone(probe.tasks[0].lock().as_ref().unwrap());
        assert_eq!(state.worker, observed.id().as_usize());
    }
    let mut running_duplicates = 0;
    for _ in 0..3 {
        running_duplicates += usize::from(!work.as_ref().queue().unwrap());
    }
    assert_eq!(running_duplicates, 3);

    // Step 3: 放行首轮回调并刷新，验证两轮回调共消费全部 16 个业务计数且最大并发度为 1。
    probe.gates[0].complete();
    work.as_ref().flush();
    queue.flush();
    let submitted = probe.submitted.load(Ordering::Acquire);
    let consumed = probe.consumed.load(Ordering::Acquire);
    let callbacks = probe.callbacks.load(Ordering::Acquire);
    let maximum = probe.maximum.load(Ordering::Acquire);
    assert_eq!((submitted, consumed, callbacks, maximum), (16, 16, 2, 1));

    // Step 4: 多核环境下利用 `PausePoint::Started` 精确拦截 Worker 刚清除 Pending 尚未进入回调的窗口，验证此时再次入队的串行重排语义。
    if let Some(started) = &started {
        testing::arm_pause(1, testing::PausePoint::Started);
        assert!(started.as_ref().queue().unwrap());
        spin_until(
            || testing::is_paused(1, testing::PausePoint::Started),
            "real claim cleared pending",
        );
        let state = testing::snapshot(started.as_ref());
        assert!(state.running && !state.queued && state.worker != 0);
        assert!(started.as_ref().queue().unwrap());
        assert!(!started.as_ref().queue().unwrap());
        let state = testing::snapshot(started.as_ref());
        assert!(state.running && state.queued);
        testing::release_pause(1, testing::PausePoint::Started);
        started.as_ref().flush();
        assert_eq!(probe.counts[1].load(Ordering::Acquire), 2);
        started.as_ref().close_sync();
    }
    work.as_ref().close_sync();
    queue.close_sync();
    probe.clear();
    unique_work(&work);
    if let Some(started) = &started {
        unique_work(started);
    }
    drop((work, started, queue, probe));
}

/// 执行者关联（Worker Association）多场景回调：
/// - 槽位 0、2：记录当前 Worker 任务并阻塞在对应门闸上，用于占用基准 Worker 或同域并发配额；
/// - 槽位 1：首轮记录原 Worker 并自我重排，次轮校验执行者仍然是同一个原 Worker；
/// - 槽位 3：在另一队列域中对槽位 1 的工作项发起跨域 `flush()`，校验不会窃取槽位 1 关联的原 Worker。
fn association_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `association_case` 持有 `Probe` 直到控制线程 `join`、全部工作项与队列同步关闭且引用计数归还。
    let (probe, index) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, work.cpu());
    assert!(!testing::is_rescuer());
    match index {
        0 | 2 => {
            probe.publish_task(index);
            probe.gates[index].wait();
        }
        1 => {
            let round = probe.counts[1].fetch_add(1, Ordering::AcqRel);
            if round == 0 {
                probe.publish_task(1);
                assert!(work.queue().unwrap());
            } else {
                assert_eq!(round, 1);
                let original = probe.tasks[1].lock().as_ref().unwrap().id();
                assert_eq!(task::current_task_id(), Some(original));
            }
        }
        3 => {
            probe.publish_task(3);
            let original = probe.tasks[1].lock().as_ref().unwrap().id();
            assert_ne!(
                task::current_task_id(),
                Some(original),
                "unrelated flush callback stole the associated executor"
            );
            probe.counts[3].store(1, Ordering::Release);
            assert!(probe.next_work(1).as_ref().flush());
            probe.counts[3].store(2, Ordering::Release);
        }
        _ => unreachable!(),
    }
}

/// 执行者关联（Worker Association）测试的并发交错动作类型。
enum AssociationAction {
    /// 正常跨域 `flush()`：验证等待同域配额释放后由原关联 Worker 继续执行第二轮。
    Flush,
    /// 原 Worker 已登记到就绪等待队列并阻塞后，取消其关联的最后一个工作项，验证动态 Worker 自动退出并被 Manager 回收。
    CancelBlocked,
    /// 原 Worker 通过首次退休检查但尚未登记到等待队列前（停在 `BeforeWait`），取消关联工作项，验证唤醒与退休无死锁。
    CancelBeforeWait,
    /// 模拟 CPU1 控制线程在持有 CPU0 停点期间发生 Panic 展开，验证 `Coordinator` 守卫自动清理停点与门闸。
    ControllerFailure,
}

/// 当 CPU1 控制线程异常展开或清理时，取消其在 CPU0 上设置的测试停点并放行阻塞门闸，防止被协调的 Worker 永久卡死。
fn release_association(probe: &Probe) {
    testing::cancel_pause(0, testing::PausePoint::Finished);
    testing::cancel_pause(0, testing::PausePoint::BeforeWait);
    for index in [0, 2, 4] {
        probe.gates[index].complete();
    }
}

/// 使用运行在 CPU1 上的独立控制线程精确编排 CPU0 上的 Worker 停点，验证执行者关联、跨域 `flush()` 与取消退休交错。
fn association_case(action: AssociationAction) -> ::ktest::TestResult {
    assert!(1 < cpu::count());
    let (cancel, before_wait, fail_controller) = match action {
        AssociationAction::Flush => (false, false, false),
        AssociationAction::CancelBlocked => (true, false, false),
        AssociationAction::CancelBeforeWait => (true, true, false),
        AssociationAction::ControllerFailure => (false, false, true),
    };
    let before = testing::pool_stats(0);
    assert_eq!(before.workers, 1);
    let domain = new_queue(WorkQueueConfig {
        max_active: 1,
        ordered: false,
        reclaim: false,
    });
    let unrelated = new_queue(WorkQueueConfig {
        max_active: 2,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let works: [Pin<Arc<Work>>; 4] = core::array::from_fn(|index| {
        probe.work(
            if index == 1 || index == 2 {
                &domain
            } else {
                &unrelated
            },
            index,
            usize::from(index == 2),
            association_callback,
        )
    });

    // Step 1: 在 CPU1 上启动协调控制线程，负责武装 CPU0 停点并验证各阶段状态。
    let controller_probe = probe.clone();
    let controller = crate::ktest::spawn_result_on(1, move |_| {
        Ok({
            let probe = controller_probe;
            let mut guard = super::Coordinator {
                owner: probe.clone(),
                release: release_association,
                success: false,
            };
            assert_eq!(cpu::current().id, 1);
            probe.gates[4].wait();
            testing::arm_pause(0, testing::PausePoint::Finished);
            assert!(probe.next_work(1).as_ref().queue().unwrap());
            until(
                || testing::is_paused(0, testing::PausePoint::Finished),
                "associated callback finished before next claim",
            );
            if fail_controller {
                panic!("association-controller-unwind");
            }
            let work = probe.next_work(1);
            let state = testing::snapshot(work.as_ref());
            assert!(state.queued && !state.running);
            assert_eq!(probe.counts[1].load(Ordering::Acquire), 1);
            let original = probe.tasks[1].lock().as_ref().unwrap().clone();
            let base = probe.tasks[0].lock().as_ref().unwrap().id();
            assert_ne!(original.id(), base);
            assert!(original.worker.is_some());

            let paused = testing::pool_stats(0);
            assert_eq!(paused.workers, before.workers + 1);
            // Step 1.1: 在同域提交槽位 2 的工作项（绑定到 CPU1），占满 `domain` 的唯一 `max_active` 配额并阻塞。
            assert_eq!(paused.created, before.created + 1);
            assert!(probe.next_work(2).as_ref().queue().unwrap());
            until(
                || probe.blocked(2),
                "remote same-domain active slot blocked",
            );
            assert!(testing::snapshot(probe.next_work(2).as_ref()).running);
            if !cancel {
                assert!(probe.next_work(3).as_ref().queue().unwrap());
            }
            if before_wait {
                testing::arm_pause(0, testing::PausePoint::BeforeWait);
            }
            // Step 1.2: 放行 `Finished` 停点，使原 Worker 尝试认领下一轮槽位 1 时因同域配额已满而推迟并进入等待路径。
            testing::release_pause(0, testing::PausePoint::Finished);
            if before_wait {
                until(
                    || testing::is_paused(0, testing::PausePoint::BeforeWait),
                    "dynamic executor passed first retirement check before ready registration",
                );
                assert!(!probe.blocked(1));
            } else {
                until(
                    || probe.blocked(1),
                    "original executor blocked with deferred associated ticket",
                );
            }
            assert!(probe.blocked(0) && probe.blocked(2));
            assert_eq!(testing::pool_stats(0).retired, before.retired);
            assert!(testing::snapshot(work.as_ref()).queued);
            assert!(!testing::snapshot(work.as_ref()).running);

            // Step 1.3: 根据测试分支执行同步取消（验证动态 Worker 无需新任务即可退出回收）或等待跨域 `flush()` 阻塞。
            if cancel {
                assert!(work.as_ref().cancel_sync());
                let state = testing::snapshot(work.as_ref());
                assert!(!state.queued && !state.running);
                assert!(probe.blocked(0) && probe.blocked(2));
                assert_eq!(probe.counts[3].load(Ordering::Acquire), 0);
                if before_wait {
                    assert!(testing::is_paused(0, testing::PausePoint::BeforeWait));
                    assert_eq!(testing::pool_stats(0).retired, before.retired);
                    assert_eq!(testing::pool_stats(0).workers, before.workers + 1);
                    testing::release_pause(0, testing::PausePoint::BeforeWait);
                }
                until(
                    || {
                        sched::test_state(&original) == State::Exited
                            && testing::pool_stats(0).retired == before.retired + 1
                            && testing::pool_stats(0).workers == before.workers
                    },
                    "cancelled last association retired and manager joined without later work",
                );
                assert_eq!(sched::test_state(&original), State::Exited);
                assert_eq!(testing::pool_stats(0).workers, before.workers);
                assert_eq!(probe.counts[1].load(Ordering::Acquire), 1);
                assert_eq!(probe.counts[3].load(Ordering::Acquire), 0);
                assert!(probe.blocked(0) && probe.blocked(2));
                assert!(testing::snapshot(probe.next_work(0).as_ref()).running);
                assert!(testing::snapshot(probe.next_work(2).as_ref()).running);
            } else {
                until(
                    || probe.blocked(3),
                    "different-domain callback really blocked in associated flush",
                );
                assert_eq!(probe.counts[3].load(Ordering::Acquire), 1);
                let flusher = probe.tasks[3].lock().as_ref().unwrap().id();
                assert_ne!(flusher, original.id());
                assert!(probe.blocked(0) && probe.blocked(2));
            }

            // Step 1.4: 放行同域占位工作项（槽位 2）与基准占位工作项（槽位 0），验证后续执行完成。
            probe.gates[2].complete();
            if !cancel {
                until(
                    || probe.counts[3].load(Ordering::Acquire) == 2,
                    "original executor continued and cross-domain flush returned",
                );
                assert_eq!(probe.counts[1].load(Ordering::Acquire), 2);
            }
            probe.gates[0].complete();
            guard.success = true;
            0
        })
    })
    .unwrap();

    // Step 2: 在 CPU0 上入队槽位 0 的工作项以占用常驻基准 Worker，随后唤醒 CPU1 控制线程开始测试交错。
    assert!(works[0].as_ref().queue().unwrap());
    until(
        || probe.blocked(0),
        "base executor held before dynamic creation",
    );
    probe.gates[4].complete();
    assert_eq!(controller.join().unwrap(), 0);
    if crate::ktest::child_failed() {
        return Err(::ktest::TestError("association controller child failed"));
    }

    for work in &works {
        work.as_ref().close_sync();
    }
    domain.close_sync();
    unrelated.close_sync();
    probe.clear();
    for work in &works {
        unique_work(work);
    }
    drop((works, domain, unrelated, probe));
    Ok(())
}

/// 深度阻塞依赖链回调：前 `DEPTH` 层依次入队下一层工作项并阻塞在各自门闸上（第 0 层额外验证先通知后 park 的早唤醒配对），最深层等待 `release` 信号后自底向上逐层唤醒。
fn chain_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `test_deep_blocking` 保持 `Probe` 存活，直到所有依赖层放行、工作项同步关闭且引用归还。
    let (probe, index) = unsafe { context(work.context()) };
    probe.publish_task(index);
    if index == 0 {
        let waiter = Waiter::new();
        assert!(waiter.waker().notify());
        waiter.park();
    }
    if index < DEPTH {
        assert!(probe.next_work(index + 1).as_ref().queue().unwrap());
        probe.gates[index].wait();
        probe.consumed.fetch_add(1, Ordering::AcqRel);
        if 0 < index {
            probe.gates[index - 1].complete();
        }
    } else {
        until(
            || probe.release.load(Ordering::Acquire),
            "deep chain release",
        );
        probe.gates[index - 1].complete();
    }
}

/// 测试多层工作项链式阻塞时动态 Worker 的按需创建、早到唤醒通知（Early Wake）配对计数以及完工后的自动退休回收。
fn test_deep_blocking() {
    let before = testing::pool_stats(0);
    let queue = new_queue(WorkQueueConfig {
        max_active: DEPTH + 1,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let works: [Pin<Arc<Work>>; DEPTH + 1] =
        core::array::from_fn(|index| probe.work(&queue, index, 0, chain_callback));
    // Step 1: 入队第 0 层工作项，等待前 `DEPTH` 层全部进入睡眠阻塞且叶子层开始执行，验证动态 Worker 已按需创建。
    assert!(works[0].as_ref().queue().unwrap());
    until(
        || (0..DEPTH).all(|index| probe.blocked(index)) && probe.tasks[DEPTH].lock().is_some(),
        "three blocked worker dependencies",
    );
    let blocked = (0..DEPTH).filter(|&index| probe.blocked(index)).count();
    let creators = testing::pool_stats(0).created - before.created;
    assert_eq!(blocked, DEPTH);
    assert!(DEPTH - 1 <= creators);

    // Step 2: 发出 `release` 放行信号并刷新队列，验证整条阻塞链自底向上全部唤醒完成。
    probe.release.store(true, Ordering::Release);
    queue.flush();
    let released = probe.consumed.load(Ordering::Acquire);
    assert_eq!(released, DEPTH);

    for work in &works {
        work.as_ref().close_sync();
    }
    queue.close_sync();
    probe.clear();
    for work in &works {
        unique_work(work);
    }
    drop((works, queue, probe));

    // Step 3: 等待所有后台服务线程重新进入空闲阻塞，核对睡眠/唤醒配对计数、早到唤醒计数与动态 Worker 退休计数。
    wait_services_blocked();
    let after = testing::pool_stats(0);
    let sleeping = after.sleeping - before.sleeping;
    let running = after.running - before.running;
    let paired = usize::from(sleeping == running && DEPTH < sleeping && after.nr_running == 0);
    let early_wake = after.early_wake - before.early_wake;
    let retired = after.retired - before.retired;
    assert_eq!(paired, 1);
    assert!(1 <= early_wake && 1 <= retired);
}

/// 按阶数从高到低分配并暂存系统中所有空闲物理页，构造物理页分配器 OOM 状态，返回暂存的物理页块总数。
fn exhaust_pages() -> usize {
    mm::drain_all_pages();
    let mut count = 0;
    for order in (0..usize::BITS as usize).rev() {
        loop {
            assert!(count < PRESSURE_CAPACITY, "work pressure page storage full");
            let Some(pages) = mm::alloc_pages(order, false) else {
                break;
            };
            // SAFETY: 仅由单线程在构造内存压力阶段顺序调用；`assert!(count < PRESSURE_CAPACITY)` 保证了下标在预分配数组边界内且该槽位尚未初始化。
            unsafe {
                pressure_pages()
                    .cast::<MaybeUninit<mm::OwnedPages>>()
                    .add(count)
                    .write(MaybeUninit::new(pages));
            }
            count += 1;
            scenario().pressure_counts.lock().0 = count;
        }
    }
    assert!(0 < count);
    assert!(mm::alloc_pages(0, false).is_none());
    count
}

/// 在物理页耗尽后遍历所有 SLUB 尺寸类别分配堆对象并暂存，耗尽 SLUB 空闲缓存直至全局分配器返回空指针，返回 `(已分配对象数, 耗尽类别数)`。
fn exhaust_objects() -> (usize, usize) {
    let mut count = 0;
    let mut nulls = 0;
    for size in mm::SLUB_SIZES {
        let layout = Layout::from_size_align(size, 8).unwrap();
        loop {
            assert!(
                count < PRESSURE_CAPACITY,
                "work pressure object storage full"
            );
            // SAFETY: `layout` 由合法的 `SLUB_SIZES` 构造（大小非零且 8 字节对齐），返回的非空指针随后立即存入压力表并在释放阶段配对归还。
            let pointer = unsafe { alloc(layout) };
            if pointer.is_null() {
                nulls += 1;
                break;
            }
            // SAFETY: 仅由单线程在构造内存压力阶段顺序调用；`assert!(count < PRESSURE_CAPACITY)` 保证了槽位在边界内且未初始化。
            unsafe {
                pressure_objects()
                    .cast::<MaybeUninit<(*mut u8, Layout)>>()
                    .add(count)
                    .write(MaybeUninit::new((pointer, layout)));
            }
            count += 1;
            scenario().pressure_counts.lock().1 = count;
        }
    }
    assert_eq!(nulls, mm::SLUB_SIZES.len());
    (count, nulls)
}

/// 从当前场景中原子取出内存压力计数并释放全部暂存的 SLUB 堆对象与物理页块。
fn release_pressure(page_count: usize, object_count: usize) {
    let actual = core::mem::take(&mut *scenario().pressure_counts.lock());
    if actual == (0, 0) && scenario().stop.load(Ordering::Acquire) {
        return;
    }
    assert_eq!(actual, (page_count, object_count));
    release_owned_pressure(page_count, object_count);
}

/// 按逆序释放前 `object_count` 个暂存的 SLUB 堆对象与前 `page_count` 个暂存的物理页块。
fn release_owned_pressure(page_count: usize, object_count: usize) {
    for index in (0..object_count).rev() {
        // SAFETY:1. 调用方已通过 `pressure_counts` 自旋锁用 `mem::take` 独占夺取了前 `object_count` 个槽位的所有权；
        // 2. 这些槽位在 `exhaust_objects` 中已完成初始化，且每个槽位仅在此处被读取并消费一次。
        let (pointer, layout) = unsafe {
            pressure_objects()
                .cast::<MaybeUninit<(*mut u8, Layout)>>()
                .add(index)
                .read()
                .assume_init()
        };
        // SAFETY: `pointer` 与 `layout` 原样来自 `exhaust_objects` 中的 `alloc(layout)` 调用，且此前未被释放或共享。
        unsafe { dealloc(pointer, layout) };
    }

    for index in (0..page_count).rev() {
        // SAFETY: 调用方已通过 `pressure_counts` 自旋锁用 `mem::take` 独占夺取了前 `page_count` 个已初始化物理页块槽位的所有权，每槽仅在此读取消费一次。
        let pages = unsafe {
            pressure_pages()
                .cast::<MaybeUninit<mm::OwnedPages>>()
                .add(index)
                .read()
                .assume_init()
        };
        drop(pages);
    }
}

/// OOM 压力测试中的占位回调：在普通 Worker 中进入睡眠阻塞，防止该常驻 Worker 认领后续的内存回收工作项。
fn pressure_blocker(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: 测试主流程在唤醒门闸、同步关闭工作项并等待子线程结束后才释放 `Probe`。
    let (probe, _) = unsafe { context(work.context()) };
    assert!(!testing::is_rescuer());
    probe.publish_task(0);
    probe.gates[0].wait();
}

/// OOM 应急救援回调：校验当前执行线程必须是队列预留的 Rescuer Worker，随后释放暂存的物理页与 SLUB 对象压力。
fn reclaim_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `test_reclaim` 在等待回收回调完成、同步关闭 `recovery` 工作项并确认引用归还后才释放 `Probe`。
    let (probe, _) = unsafe { context(work.context()) };
    if testing::is_rescuer() {
        probe.rescuer.fetch_add(1, Ordering::AcqRel);
    } else {
        probe.ordinary.fetch_add(1, Ordering::AcqRel);
    }
    assert!(
        testing::is_rescuer(),
        "ordinary worker bypassed real OOM rescue"
    );
    let pages = probe.pages.load(Ordering::Acquire);
    let objects = probe.objects.load(Ordering::Acquire);
    assert!(0 < pages);
    release_pressure(pages, objects);
    probe.done.store(true, Ordering::Release);
}

/// 测试带 `reclaim` 标志的工作队列在物理页与全部 SLUB 堆缓存耗尽导致新 Worker 创建失败时，唤醒预留的 Rescuer 线程执行回收回调并恢复内存分配。
fn test_reclaim() {
    let queue = new_queue(WorkQueueConfig {
        max_active: 4,
        ordered: false,
        reclaim: true,
    });
    let probe = Probe::new();
    let blocker = probe.work(&queue, 0, 0, pressure_blocker);
    let recovery = probe.work(&queue, 1, 0, reclaim_callback);
    // Step 1: 让占位工作项阻塞唯一的常驻普通 Worker，使空闲普通 Worker 数量降为 0。
    assert!(blocker.as_ref().queue().unwrap());
    until(|| probe.blocked(0), "ordinary worker blocked before OOM");
    let before = testing::pool_stats(0);
    assert_eq!(before.workers, 1);
    assert_eq!(
        before.idle, 0,
        "ordinary idle worker could bypass OOM rescue"
    );

    // Step 2: 收缩缓存并耗尽物理页与所有 SLUB 规格对象，随后入队 `recovery` 回收工作项。
    mm::shrink_all();
    let page_count = exhaust_pages();
    let (objects, _heap_null) = exhaust_objects();
    let page_null = usize::from(mm::alloc_pages(0, false).is_none());
    assert_eq!(page_null, 1);
    probe.pages.store(page_count, Ordering::Release);
    probe.objects.store(objects, Ordering::Release);
    assert!(recovery.as_ref().queue().unwrap());
    // Step 3: 等待预留的 Rescuer 线程执行回收回调并释放内存压力，验证普通 Worker 创建失败计数与 Rescuer 救援计数递增。
    until(
        || probe.done.load(Ordering::Acquire),
        "reserved rescuer real OOM progress",
    );
    recovery.as_ref().flush();
    let after = testing::pool_stats(0);
    let create_failures = after.create_failures - before.create_failures;
    let rescuer_callbacks = probe.rescuer.load(Ordering::Acquire);
    let worker_callbacks = probe.ordinary.load(Ordering::Acquire);
    assert!(1 <= create_failures && 1 <= after.rescued - before.rescued);
    assert_eq!((rescuer_callbacks, worker_callbacks), (1, 0));

    // Step 4: 验证物理页分配器与全部 SLUB 规格缓存均已恢复正常分配能力。
    let recovered_page = mm::alloc_pages(0, false).expect("page allocation did not recover");
    drop(recovered_page);
    let mut heap_recovered = 0;
    for size in mm::SLUB_SIZES {
        let layout = Layout::from_size_align(size, 8).unwrap();
        // SAFETY: `layout` 来自合法的 `SLUB_SIZES`（非零且对齐），用于验证该规格堆分配已恢复。
        let pointer = unsafe { alloc(layout) };
        assert!(!pointer.is_null(), "heap class did not recover");
        heap_recovered += usize::from(!pointer.is_null());
        // SAFETY: `pointer` 由上方紧邻的 `alloc(layout)` 成功分配，使用相同 `layout` 立即释放。
        unsafe { dealloc(pointer, layout) };
    }
    let recovered = usize::from(heap_recovered == mm::SLUB_SIZES.len());
    assert_eq!(recovered, 1);

    // Step 5: 放行阻塞的普通 Worker 并同步关闭所有工作项与队列。
    probe.gates[0].complete();
    queue.flush();
    blocker.as_ref().close_sync();
    recovery.as_ref().close_sync();
    queue.close_sync();
    probe.clear();
    unique_work(&blocker);
    unique_work(&recovery);
    drop((blocker, recovery, queue, probe));
}

/// 在给定的硬件时钟截止时间 `end` 前循环等待条件 `predicate` 成立，期间主动让出 CPU。
fn ordinary_until(end: u64, mut predicate: impl FnMut() -> bool, name: &str) {
    while !predicate() {
        assert!(arch::read_counter() < end, "ordinary OOM timeout: {}", name);
        sched::yield_now();
    }
}

/// 在 1.25 秒的观测窗口内持续检查普通队列遭遇 OOM 后的冷却行为：验证 Manager 线程进入阻塞等待，且重试失败次数受控（取消工作项后不再重试）。
fn ordinary_creation_window(
    probe: &Probe,
    pending: Pin<&Work>,
    queued: bool,
    failures: usize,
    maximum: usize,
    end: u64,
) {
    let window_end = arch::read_counter()
        .checked_add(arch::counter_frequency().checked_mul(1250).unwrap() / 1000)
        .unwrap();
    let mut blocked = false;
    while arch::read_counter() < window_end {
        assert!(
            arch::read_counter() < end,
            "ordinary OOM observation timeout"
        );
        assert!(probe.blocked(0), "base worker escaped its completion wait");
        assert!(
            probe.blocked(2),
            "independent release task ran before permission"
        );
        assert!(!probe.release.load(Ordering::Acquire));
        assert_eq!(probe.ordinary.load(Ordering::Acquire), 0);
        let snapshot = testing::snapshot(pending);
        assert_eq!((snapshot.queued, snapshot.running), (queued, false));
        let stats = testing::pool_stats(0);
        assert_eq!((stats.workers, stats.idle, stats.nr_running), (1, 0, 0));
        assert!(
            stats.create_failures - failures <= maximum,
            "worker creation spun during OOM or continued after cancellation"
        );
        blocked |= testing::manager_blocked(0);
        sched::yield_now();
    }
    assert!(blocked, "manager never entered a real Blocked state");
}

/// 预先创建的独立内存释放线程入口：在门闸唤醒后释放全部暂存的物理页与 SLUB 对象，验证分配恢复后通知工作队列并继续睡眠。
fn ordinary_release_task(probe: Arc<Probe>) -> i32 {
    assert!(task::current_worker().is_none());
    probe.publish_task(2);
    probe.gates[2].wait();
    if scenario().stop.load(Ordering::Acquire) {
        return 20;
    }
    let pages = probe.pages.load(Ordering::Acquire);
    let objects = probe.objects.load(Ordering::Acquire);
    assert!(0 < pages);
    release_pressure(pages, objects);
    let stack = mm::alloc_pages(4, false).expect("independent release did not recover stack pages");
    drop(stack);
    for size in mm::SLUB_SIZES {
        let layout = Layout::from_size_align(size, 8).unwrap();
        // SAFETY: `layout` 来自合法的 `SLUB_SIZES`（非零且对齐），用于验证该规格堆分配已恢复。
        let pointer = unsafe { alloc(layout) };
        assert!(
            !pointer.is_null(),
            "independent release did not recover heap class"
        );
        // SAFETY: `pointer` 由上方紧邻的 `alloc(layout)` 成功分配，使用相同 `layout` 立即释放。
        unsafe { dealloc(pointer, layout) };
    }
    // 当前线程绑定在 CPU0 上，释放内存并设置 `release` 标志后立即阻塞在门闸 3 上，将 CPU0 让给工作队列 Manager 线程继续创建新 Worker。
    probe.release.store(true, Ordering::Release);
    probe.gates[3].wait();
    20
}

/// 普通队列 OOM 恢复后的测试回调：验证其在独立线程释放内存压力后由新创建的普通 Worker 执行，并唤醒一直阻塞的基础占位任务。
fn ordinary_recovery_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `ordinary_recovery_case` 保持 `Probe` 存活，直到所有工作项同步关闭、独立释放线程 `join` 且引用归还。
    let (probe, _) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, 0);
    assert!(task::current_worker().is_some());
    assert!(!testing::is_rescuer());
    assert!(
        probe.release.load(Ordering::Acquire),
        "ordinary B ran before independent release"
    );
    assert!(probe.blocked(0));
    let base_id = probe.tasks[0].lock().as_ref().unwrap().id();
    assert_ne!(task::current_task().id(), base_id);
    probe.publish_task(1);
    assert_eq!(probe.ordinary.fetch_add(1, Ordering::AcqRel), 0);
    probe.done.store(true, Ordering::Release);
    probe.gates[0].complete();
}

/// 测试普通工作队列（无 Rescuer）在 OOM 导致动态 Worker 创建失败并进入冷却期后，在外部独立释放内存后自动恢复执行（或冷却期内取消工作项后停止创建重试）。
fn ordinary_recovery_case(cancel: bool) -> (usize, usize) {
    let queue = new_queue(WorkQueueConfig {
        max_active: 2,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let blocker = probe.work(&queue, 0, 0, pressure_blocker);
    let pending = probe.work(&queue, 1, 0, ordinary_recovery_callback);
    // Step 1: 在 OOM 发生前预先启动独立内存释放线程并使其阻塞就绪，同时让 `blocker` 工作项占满唯一的常驻普通 Worker。
    let release_probe = Arc::clone(&probe);
    let prepared =
        crate::ktest::spawn_result_on(0, move |_| Ok(ordinary_release_task(release_probe)))
            .expect("prepare independent ordinary release task");
    let release_task = prepared;
    until(
        || probe.blocked(2),
        "independent release task prepared and parked",
    );
    assert!(blocker.as_ref().queue().unwrap());
    until(
        || probe.blocked(0),
        "base worker blocked before ordinary OOM",
    );
    let before = testing::pool_stats(0);
    assert_eq!((before.workers, before.idle, before.nr_running), (1, 0, 0));

    // Step 2: 耗尽物理页与所有 SLUB 缓存，提交 `pending` 工作项触发 Manager 创建新 Worker 失败并进入冷却阻塞。
    mm::shrink_all();
    let pages = exhaust_pages();
    let (objects, nulls) = exhaust_objects();
    assert_eq!(nulls, mm::SLUB_SIZES.len());
    assert!(mm::alloc_pages(0, false).is_none());
    probe.pages.store(pages, Ordering::Release);
    probe.objects.store(objects, Ordering::Release);
    assert!(pending.as_ref().queue().unwrap());
    let end = raw_deadline();
    ordinary_until(
        end,
        || {
            before.create_failures < testing::pool_stats(0).create_failures
                && testing::manager_blocked(0)
        },
        "real ordinary create failure followed by manager Blocked",
    );
    let failed = testing::pool_stats(0).create_failures;
    // Step 3: 根据 `cancel` 参数决定是在冷却期内取消 `pending` 工作项还是保留等待，并在 1.25 秒窗口内验证 Manager 不发生忙等自旋。
    if cancel {
        assert!(pending.as_ref().cancel_sync());
        ordinary_creation_window(&probe, pending.as_ref(), false, failed, 0, end);
    } else {
        ordinary_creation_window(&probe, pending.as_ref(), true, failed, 2, end);
    }
    let failures = testing::pool_stats(0).create_failures - before.create_failures;
    assert!(1 <= failures);

    // Step 4: 唤醒独立线程释放内存压力，验证未取消时 Manager 自动创建新 Worker 完成回调（无需新的入队事件触发），已取消时则不创建新 Worker。
    probe.gates[2].complete();
    ordinary_until(
        end,
        || probe.release.load(Ordering::Acquire),
        "independent task released real page and heap pressure",
    );
    if cancel {
        assert!(probe.blocked(0));
        assert_eq!(probe.ordinary.load(Ordering::Acquire), 0);
        assert!(!probe.done.load(Ordering::Acquire));
        assert_eq!(testing::pool_stats(0).created, before.created);
        probe.gates[0].complete();
    } else {
        ordinary_until(
            end,
            || probe.done.load(Ordering::Acquire),
            "accepted B stalled after independent memory release without new work",
        );
        assert_eq!(probe.ordinary.load(Ordering::Acquire), 1);
        assert!(before.created < testing::pool_stats(0).created);
    }

    // Step 5: 刷新并同步关闭所有工作项、队列与辅助线程，返回创建失败次数与回调执行次数。
    pending.as_ref().flush();
    queue.flush();
    probe.gates[3].complete();
    assert_eq!(release_task.join().unwrap(), 20);
    blocker.as_ref().close_sync();
    pending.as_ref().close_sync();
    queue.close_sync();
    let callbacks = probe.ordinary.load(Ordering::Acquire);
    assert_eq!(probe.rescuer.load(Ordering::Acquire), 0);
    probe.clear();
    unique_work(&blocker);
    unique_work(&pending);
    drop((blocker, pending, queue, probe));
    (failures, callbacks)
}

/// `flush()` 边界测试回调：槽位 0 在绑定 CPU 上阻塞，槽位 1 立即完成，槽位 2 在生产源开启时自我重排 4 次后阻塞，槽位 3..5 直接阻塞在门闸上。
fn boundary_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `test_flush_boundary` 在关闭生产源、放行所有门闸并同步关闭全部工作项后才释放 `Probe`。
    let (probe, index) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, work.cpu());
    if index == 0 {
        probe.publish_task(0);
        probe.gates[0].wait();
        probe.counts[0].fetch_add(1, Ordering::Release);
    } else if index == 1 {
        probe.counts[1].fetch_add(1, Ordering::Release);
    } else if index == 2 && probe.counts[6].load(Ordering::Acquire) < 4 {
        if scenario().stop.load(Ordering::Acquire) {
            return;
        }
        assert!(probe.source.load(Ordering::Acquire));
        assert!(work.queue().unwrap());
        probe.counts[6].fetch_add(1, Ordering::AcqRel);
    } else {
        probe.publish_task(index);
        probe.gates[index].wait();
        probe.counts[index].fetch_add(1, Ordering::Release);
    }
}

/// 测试队列 `flush()` 的批次颜色边界语义：验证 `flush()` 仅等待调用时刻已入队的旧工作项完成，而不会被 `flush()` 阻塞期间新提交的未来工作项无限推迟。
fn test_flush_boundary() {
    let queue = new_queue(WorkQueueConfig {
        max_active: SLOTS,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let early_cpu = if cpu::count() == 1 { 0 } else { 1 };
    let works: [Pin<Arc<Work>>; 6] = core::array::from_fn(|index| {
        probe.work(
            &queue,
            index,
            if index == 0 { early_cpu } else { 0 },
            boundary_callback,
        )
    });

    // Step 1: 先入队槽位 0 使其在目标 CPU 上睡眠阻塞，再入队槽位 1 并等待其执行完毕。
    assert!(works[0].as_ref().queue().unwrap());
    until(|| probe.blocked(0), "earlier work sleeping on bound CPU");
    let _earlier_blocked = usize::from(probe.blocked(0));
    let remote = usize::from(works[0].cpu() != cpu::current().id);
    assert_eq!(remote, usize::from(1 < cpu::count()));
    assert!(works[1].as_ref().queue().unwrap());
    until(
        || probe.counts[1].load(Ordering::Acquire) == 1,
        "later work completed",
    );
    let _later_done = probe.counts[1].load(Ordering::Acquire);

    // Step 2: 启动刷新线程调用 `queue.flush()`，验证其因槽位 0 尚未完成而进入阻塞等待。
    let flush_probe = probe.clone();
    let queue_address = Arc::as_ptr(&queue) as usize;
    let flusher = crate::ktest::spawn_result_on(0, move |_| {
        Ok({
            let probe = flush_probe;
            probe.publish_task(7);
            probe.started.store(true, Ordering::Release);
            // SAFETY: 外层测试主线程持有 `queue` 的 `Arc<QueueOwner>` 直至 `flusher.join()` 返回后才释放，此处转为共享引用调用 `flush()` 期间对象始终有效。
            let queue = unsafe { &*(queue_address as *const super::QueueOwner) };
            queue.flush();
            probe.done.store(true, Ordering::Release);
            21
        })
    })
    .unwrap();
    until(
        || probe.started.load(Ordering::Acquire) && probe.blocked(7),
        "flush old boundary actually blocked",
    );
    assert!(!probe.done.load(Ordering::Acquire));
    assert_eq!(probe.counts[0].load(Ordering::Acquire), 0);

    // Step 3: 在 `flush()` 阻塞期间提交槽位 2..6 的未来工作项（含槽位 2 的 4 次自我重排），并使它们停留在 `queued` 或 `running` 状态。
    let mut future_submitted = 0;
    for work in &works[2..] {
        future_submitted += usize::from(work.as_ref().queue().unwrap());
    }
    assert_eq!(future_submitted, 4);
    until(
        || probe.counts[6].load(Ordering::Acquire) == 4,
        "finite future callback production",
    );
    let _future_submitted = future_submitted + probe.counts[6].load(Ordering::Acquire);
    probe.source.store(false, Ordering::Release);
    until(
        || {
            works[2..].iter().all(|work| {
                let state = testing::snapshot(work.as_ref());
                state.queued || state.running
            })
        },
        "future accepted work retained",
    );

    // Step 4: 放行旧批次中阻塞的槽位 0，验证 `flush()` 立即返回，无需等待仍处于活跃或排队状态的未来工作项（槽位 2..6）。
    probe.gates[0].complete();
    until(
        || probe.done.load(Ordering::Acquire),
        "finite old color flush returned",
    );
    assert_eq!(flusher.join().unwrap(), 21);
    let future_running = works[2..]
        .iter()
        .filter(|work| {
            let state = testing::snapshot(work.as_ref());
            state.running || state.queued
        })
        .count();
    let boundary_done = usize::from(
        probe.counts[0].load(Ordering::Acquire) == 1 && probe.done.load(Ordering::Acquire),
    );
    assert_eq!(boundary_done, 1);
    assert!(1 <= future_running);

    // Step 5: 放行所有未来工作项并完成最终清理。
    for index in 2..6 {
        probe.gates[index].complete();
    }
    queue.flush();
    assert!((0..6).all(|index| probe.counts[index].load(Ordering::Acquire) == 1));
    for work in &works {
        work.as_ref().close_sync();
    }
    queue.close_sync();
    probe.clear();
    for work in &works {
        unique_work(work);
    }
    drop((works, queue, probe));
}

/// `DelayedWork` 测试回调：若检测到关闭后标志 `delay_source_closed` 则记录违规访问；否则验证运行在绑定的目标 CPU Worker 线程上下文（非软中断下半部）并累加回调计数。
fn delayed_callback(work: Pin<&DelayedWork>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    if scenario().delay_source_closed.load(Ordering::Acquire) {
        scenario().access_after_close.fetch_add(1, Ordering::AcqRel);
        return;
    }
    // SAFETY: 调用方测试用例在显式关闭 `DelayedWork` 并确认异步引用全部归还后才释放 `Probe`。
    let (probe, _) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, work.cpu());
    assert_eq!(cpu::current().bh_depth.load(Ordering::Relaxed), 0);
    let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
    probe.maximum.fetch_max(active, Ordering::AcqRel);
    assert_eq!(active, 1);
    probe.callbacks.fetch_add(1, Ordering::Release);
    assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// 测试远端 CPU 定时器到期将 `DelayedWork` 移交至工作队列的前后停点窗口内，同步取消（`cancel_sync`）必须阻塞等待移交尾部彻底完成。
fn paused_delayed_stop(
    work: &Pin<Arc<DelayedWork>>,
    probe: &Arc<Probe>,
    point: delayed_testing::PausePoint,
) -> (usize, usize) {
    assert!(1 < cpu::count());
    probe.started.store(false, Ordering::Relaxed);
    probe.done.store(false, Ordering::Relaxed);
    let stop_work = work.clone();
    let stop_probe = probe.clone();
    // Step 1: 在 CPU0 上启动取消线程并使其在门闸 7 上待命。
    let handle = crate::ktest::spawn_result_on(0, move |_| {
        Ok({
            let probe = stop_probe;
            probe.publish_task(7);
            probe.gates[7].wait();
            probe.started.store(true, Ordering::Release);
            stop_work.as_ref().cancel_sync();
            let state = delayed_testing::snapshot(stop_work.as_ref());
            assert!(!state.running && !state.timer_pending && !state.work_pending);
            probe.done.store(true, Ordering::Release);
            22
        })
    })
    .unwrap();
    until(|| probe.blocked(7), "delayed stopper prepared");

    // Step 2: 配置 `DelayedWork` 在定时器移交停点 `point` 暂停并入队，自旋等待远端 CPU1 到达该停点后唤醒 CPU0 取消线程。
    delayed_testing::configure(work.as_ref(), point);
    assert!(work.as_ref().queue(Duration::from_millis(10)).unwrap());
    spin_until(
        delayed_testing::arrived,
        "real timer to pool transfer pause",
    );
    let inflight = usize::from(delayed_testing::arrived());
    probe.gates[7].complete();
    until(
        || probe.started.load(Ordering::Acquire) && probe.blocked(7),
        "delayed sync cancel waiting real timer tail",
    );
    let sync_wait = usize::from(probe.blocked(7));
    assert!(!probe.done.load(Ordering::Acquire));
    assert!(delayed_testing::arrived());

    // Step 3: 验证取消线程在停点释放前处于阻塞等待状态，随后释放停点并确认取消线程顺利完成退出。
    delayed_testing::release();
    until(
        || handle.try_join().is_some(),
        "delayed cancel tail returned",
    );
    assert_eq!(handle.join().unwrap(), 22);
    assert!(probe.done.load(Ordering::Acquire));
    let task = probe.tasks[7].lock().take().unwrap();
    assert_eq!(sched::test_state(&task), State::Exited);
    assert_eq!(Arc::strong_count(&task), 1);
    drop(task);
    assert_eq!((sync_wait, inflight), (1, 1));
    (sync_wait, inflight)
}

/// 测试工作队列先行关闭（`close_sync`）后，已入队的 `DelayedWork` 定时器到期移交被拒绝时，必须自动清除 Pending 状态并归还异步强引用。
fn rejected_delayed_transfer(
    queue: &super::QueueOwner,
    work: &Pin<Arc<DelayedWork>>,
    probe: &Probe,
) -> usize {
    let callbacks = probe.callbacks.load(Ordering::Acquire);
    if 1 < cpu::count() {
        delayed_testing::configure(work.as_ref(), delayed_testing::PausePoint::BeforeTransfer);
        assert!(work.as_ref().queue(Duration::from_millis(10)).unwrap());
        spin_until(delayed_testing::arrived, "closed domain due timer paused");
        let pending = delayed_testing::snapshot(work.as_ref());
        assert!(pending.timer_pending && !pending.work_pending && !pending.running);
        queue.close_sync();
        assert!(delayed_testing::arrived());
        delayed_testing::release();
    } else {
        assert!(work.as_ref().queue(Duration::from_secs(1)).unwrap());
        assert!(delayed_testing::snapshot(work.as_ref()).timer_pending);
        queue.close_sync();
        // 单核环境下通过 `timer::sleep` 睡眠 1 秒等待定时器到期，随后在未调用 `DelayedWork` 取消/关闭的前提下验证其自动清理状态并归还引用。
        crate::timer::sleep(Duration::from_secs(1));
    }
    until(
        || {
            let state = delayed_testing::snapshot(work.as_ref());
            !state.timer_pending && !state.work_pending && !state.running
        },
        "closed domain rejected due timer removed pending",
    );
    let orphan = unique_delayed(work);
    // 等待定时器后台服务重新进入空闲阻塞，确认到期处理路径已完全退出。
    super::timer::wait_services_blocked();
    assert_eq!(probe.callbacks.load(Ordering::Acquire), callbacks);
    // 以上所有状态与引用计数断言均在调用 `close_delayed` 之前完成，确保到期被拒分支自身完成了清理而非依赖后续关闭操作兜底。
    scenario().nodes.close_delayed(work.as_ref());
    assert!(delayed_testing::snapshot(work.as_ref()).closed);
    orphan
}

/// 测试同一个 `DelayedWork` 的延迟入队、到期时间修改（`modify`）、两次到期移交执行、同步取消（`cancel_sync`）以及关闭后的拒绝入队与零泄漏保证。
fn test_delayed() {
    scenario()
        .delay_source_closed
        .store(false, Ordering::Release);
    let queue = new_queue(WorkQueueConfig {
        max_active: 4,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let owner = if cpu::count() == 1 { 0 } else { 1 };
    let work = new_delayed(
        &queue,
        owner,
        delayed_callback,
        &probe.entries[0] as *const Entry as usize,
    );

    let mut delayed = usize::from(work.as_ref().queue(Duration::from_secs(1)).unwrap());
    assert!(delayed_testing::snapshot(work.as_ref()).timer_pending);
    let modified = usize::from(work.as_ref().modify(Duration::from_millis(10)).unwrap());
    assert_eq!(modified, 1);
    assert!(work.as_ref().flush());
    assert_eq!(probe.callbacks.load(Ordering::Acquire), 1);
    delayed += usize::from(work.as_ref().queue(Duration::from_millis(10)).unwrap());
    until(
        || probe.callbacks.load(Ordering::Acquire) == 2,
        "second delayed timer transfer callback",
    );
    work.as_ref().flush();
    let transfers = probe.callbacks.load(Ordering::Acquire);
    assert_eq!((delayed, transfers), (2, 2));

    assert!(work.as_ref().queue(Duration::from_secs(1)).unwrap());
    let cancelled = usize::from(work.as_ref().cancel_sync());
    assert_eq!(cancelled, 1);
    let idle = delayed_testing::snapshot(work.as_ref());
    assert!(!idle.timer_pending && !idle.work_pending && !idle.running);

    scenario().nodes.close_delayed(work.as_ref());
    scenario()
        .delay_source_closed
        .store(true, Ordering::Release);
    assert_eq!(
        work.as_ref().queue(Duration::from_millis(1)),
        Err(DelayedError::Closed)
    );
    let closed = delayed_testing::snapshot(work.as_ref());
    assert!(closed.closed && !closed.running && !closed.timer_pending && !closed.work_pending);
    queue.close_sync();
    let orphan = unique_delayed(&work);
    assert_eq!(orphan, 0);
    probe.clear();
    drop((work, queue, probe));
    crate::timer::sleep(Duration::from_millis(30));
    let access_after_close = scenario().access_after_close.load(Ordering::Acquire);
    assert_eq!(access_after_close, 0);
}

/// 模拟持有工作项与业务状态所有权的顶层业务结构（用于验证显式 `close()` 与 `Drop` 职责分离）。
struct SourceOwner {
    /// 关联的探测与同步控制块。
    probe: Arc<Probe>,
    /// 受管理的底层工作项。
    work: Pin<Arc<Work>>,
}

impl SourceOwner {
    /// 先关闭业务层的继续生产标志，再对工作项执行一次性同步关闭（`close_sync`）。
    fn close(&self) {
        assert_eq!(self.probe.stops.load(Ordering::Acquire), 0);
        self.probe.source.store(false, Ordering::Release);
        self.work.as_ref().close_sync();
        assert_eq!(self.probe.stops.fetch_add(1, Ordering::AcqRel), 0);
    }
}

impl Drop for SourceOwner {
    /// 校验在析构前已完成显式同步关闭且无活跃回调，并累加场景级的 `SourceOwner` 析构计数。
    fn drop(&mut self) {
        assert_eq!(self.probe.stops.load(Ordering::Acquire), 1);
        assert!(testing::snapshot(self.work.as_ref()).closed);
        assert_eq!(self.probe.active.load(Ordering::Acquire), 0);
        scenario().owner_drops.fetch_add(1, Ordering::AcqRel);
    }
}

/// 自我重排（Self-requeue）测试回调：前两轮在回调运行期间再次将自身入队（使 `running` 与 `queued` 并存但串行执行）。
fn requeue_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: `SourceOwner` 同时持有 `Probe` 与 `Work`，且在 `close()` 同步等待回调彻底结束后才释放 `Probe`。
    let (probe, _) = unsafe { context(work.context()) };
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    assert!(probe.source.load(Ordering::Acquire));
    let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
    probe.maximum.fetch_max(active, Ordering::AcqRel);
    assert_eq!(active, 1);
    let round = probe.callbacks.fetch_add(1, Ordering::AcqRel);
    if round < 2 {
        assert!(work.queue().unwrap());
        let state = testing::snapshot(work);
        assert!(state.running && state.queued);
        probe.submitted.fetch_add(1, Ordering::AcqRel);
    }
    sched::yield_now();
    assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// 测试工作项的自我重排、同步取消后的再次入队复用，以及 `SourceOwner` 显式同步关闭与永久拒绝后续入队的生命周期契约。
fn test_owner() {
    let queue = new_queue(WorkQueueConfig {
        max_active: 4,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let work = probe.work(&queue, 0, 0, requeue_callback);
    let owner = Arc::new(SourceOwner { probe, work });
    scenario().owners.lock().push(owner.clone());
    assert!(owner.work.as_ref().queue().unwrap());
    until(
        || owner.probe.callbacks.load(Ordering::Acquire) == 3,
        "self requeue real callbacks",
    );
    owner.work.as_ref().cancel_sync();
    let idle = testing::snapshot(owner.work.as_ref());
    assert!(!idle.running && !idle.queued && !idle.closed);

    let reused = usize::from(owner.work.as_ref().queue().unwrap());
    assert_eq!(reused, 1);
    owner.work.as_ref().flush();
    let callbacks = owner.probe.callbacks.load(Ordering::Acquire);
    let self_requeue = owner.probe.submitted.load(Ordering::Acquire);
    let max_active = owner.probe.maximum.load(Ordering::Acquire);
    assert_eq!((self_requeue, callbacks, max_active), (2, 4, 1));

    owner.close();
    let closed = usize::from(testing::snapshot(owner.work.as_ref()).closed);
    let rejected = usize::from(owner.work.as_ref().queue() == Err(QueueError::Closed));
    let stops = owner.probe.stops.load(Ordering::Acquire);
    assert_eq!((closed, rejected, stops), (1, 1, 1));

    queue.close_sync();
    owner.probe.clear();
    unique_work(&owner.work);
    drop((owner, queue));
}

/// 持续自我重排测试回调：在首轮唤醒门闸 0 后，只要 `source` 为 `true` 就持续将自身重新入队（期间不主动睡眠或让出），直到同核普通线程将 `source` 置为 `false` 后通知门闸 1。
fn continuous_requeue_callback(work: Pin<&Work>) {
    let owner = scenario();
    if owner.stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: 外层测试函数与夹具持有 `Probe` 直到工作项同步关闭且对端线程 `join` 返回，借用仅在当前回调内有效。
    let (probe, _) = unsafe { context(work.context()) };
    if probe.callbacks.fetch_add(1, Ordering::AcqRel) == 0 {
        probe.gates[0].complete();
    }
    if probe.source.load(Ordering::Acquire) {
        let queued = work.queue();
        if !owner.stop.load(Ordering::Acquire) {
            assert_eq!(queued, Ok(true));
        }
    } else {
        probe.consumed.fetch_add(1, Ordering::AcqRel);
        probe.gates[1].complete();
    }
}

/// 测试持续自我重排（Self-requeue）的工作项不会饿死同 CPU 上的普通线程：验证 Worker 在回调边界响应调度让出 CPU，使同核普通线程得以运行并关闭生产源。
fn test_continuous_peer_progress() -> ::ktest::TestResult {
    let queue = new_queue(WorkQueueConfig {
        max_active: 1,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let work = probe.work(&queue, 0, 0, continuous_requeue_callback);
    let peer_probe = probe.clone();
    let peer = crate::ktest::spawn_result_on(0, move |_| {
        peer_probe.gates[0].wait();
        if scenario().stop.load(Ordering::Acquire) {
            return Ok(0);
        }
        assert!(peer_probe.source.swap(false, Ordering::AcqRel));
        Ok(0)
    })?;
    assert!(work.as_ref().queue().unwrap());
    assert_eq!(peer.join()?, 0);
    probe.gates[1].wait();
    work.as_ref().close_sync();
    queue.close_sync();
    assert_eq!(probe.consumed.load(Ordering::Acquire), 1);
    probe.clear();
    unique_work(&work);
    drop((work, queue, probe));
    Ok(())
}

/// 回调交接（Handoff）测试回调：槽位 0 入队未关联的槽位 1 并触发本核重调度请求（`KICK_RESCHEDULE`），槽位 1 执行完成后唤醒门闸 1。
fn handoff_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: 测试主函数与夹具保持 `Probe` 存活直至两个工作项均完成同步关闭，借用不逃逸出当前回调。
    let (probe, index) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, 0);
    assert_eq!(probe.counts[index].fetch_add(1, Ordering::AcqRel), 0);
    if index == 0 {
        assert!(probe.next_work(1).as_ref().queue().unwrap());
        cpu::kick(0, cpu::KICK_RESCHEDULE);
        spin_until(
            || cpu::current().need_resched.load(Ordering::Relaxed),
            "callback handoff reschedule request",
        );
    } else {
        assert_eq!(index, 1);
        probe.gates[1].complete();
    }
}

/// 测试普通 Worker 在完成当前回调并在边界让出 CPU 后，被重新调度回来时能够直接认领队列中未关联的后继工作项，无需为此额外创建新的动态 Worker。
fn test_callback_handoff() {
    let queue = new_queue(WorkQueueConfig {
        max_active: 1,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let first = probe.work(&queue, 0, 0, handoff_callback);
    let next = probe.work(&queue, 1, 0, handoff_callback);

    wait_services_blocked();
    let created = testing::pool_stats(0).created;
    assert!(first.as_ref().queue().unwrap());
    probe.gates[1].wait();
    first.as_ref().close_sync();
    next.as_ref().close_sync();
    queue.close_sync();
    assert_eq!(probe.counts[0].load(Ordering::Acquire), 1);
    assert_eq!(probe.counts[1].load(Ordering::Acquire), 1);
    probe.clear();
    unique_work(&first);
    unique_work(&next);
    drop((first, next, queue, probe));
    wait_services_blocked();
    assert_eq!(testing::pool_stats(0).created, created);
}

/// 测试工作队列关闭后拒绝到期 `DelayedWork` 移交并自动归还异步引用的完整流程。
fn test_rejected_transfer() {
    let probe = Probe::new();
    let owner = if cpu::count() == 1 { 0 } else { 1 };
    let rejected_queue = new_queue(WorkQueueConfig {
        max_active: 1,
        ordered: false,
        reclaim: false,
    });
    let rejected_work = new_delayed(
        &rejected_queue,
        owner,
        delayed_callback,
        &probe.entries[1] as *const Entry as usize,
    );
    assert_eq!(
        rejected_delayed_transfer(&rejected_queue, &rejected_work, &probe),
        0
    );
    probe.clear();
}

/// 单个工作队列测试用例（或异常恢复场景）的全局夹具状态容器。
struct Scenario {
    /// 统一追踪本用例创建的所有工作队列、`Work` 与 `DelayedWork` 节点的容器。
    nodes: super::Nodes,
    /// 通知所有正在运行的测试回调或辅助线程立即中止的原子标志。
    stop: AtomicBool,
    /// 已完成 `Drop` 析构的 [`Probe`] 实例计数。
    operation_drops: AtomicUsize,
    /// `DelayedWork` 在 `delay_source_closed` 置位后仍进入回调的违规访问计数。
    access_after_close: AtomicUsize,
    /// 标记 `DelayedWork` 业务源已关闭的哨兵标志。
    delay_source_closed: AtomicBool,
    /// 已完成 `Drop` 析构的 [`SourceOwner`] 实例计数。
    owner_drops: AtomicUsize,
    /// 本用例创建的所有 [`Probe`] 强引用列表，用于在清理阶段统一放行门闸并核对析构数量。
    probes: SpinLock<alloc::vec::Vec<Arc<Probe>>>,
    /// 本用例创建的所有 [`SourceOwner`] 强引用列表，用于在清理阶段确保全部完成显式关闭并核对析构数量。
    owners: SpinLock<alloc::vec::Vec<Arc<SourceOwner>>>,
    /// OOM 压力测试预分配的物理页块暂存数组（容量在 `set_up` 中固定，不再扩容）。
    pressure: core::cell::UnsafeCell<alloc::vec::Vec<MaybeUninit<mm::OwnedPages>>>,
    /// OOM 压力测试预分配的 SLUB 堆对象指针与布局暂存数组（容量在 `set_up` 中固定，不再扩容）。
    objects: core::cell::UnsafeCell<alloc::vec::Vec<MaybeUninit<(*mut u8, Layout)>>>,
    /// 当前暂存的 `(物理页块数量, SLUB 堆对象数量)`，由自旋锁保护以确保唯一回收者夺取所有权。
    pressure_counts: SpinLock<(usize, usize)>,
}

/// 指向当前正在运行的测试用例 [`Scenario`] 实例的全局原子指针。
static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 获取当前活动测试场景 [`Scenario`] 的强引用克隆。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY: `CURRENT` 在 `ServiceFixture::set_up` 中通过 `Arc::into_raw` 登记，并在所有异步回调与子线程清理完毕后的 `tear_down` 中才置空并释放；期间指向合法的 `Scenario` 实例。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}

/// 工作队列测试套件的用例级夹具（Fixture），负责管理单次测试的 [`Scenario`] 状态与系统资源基线快照。
pub(super) struct ServiceFixture {
    /// 当前测试用例的场景状态容器。
    state: Arc<Scenario>,
    /// 测试用例启动前记录的内存与工作队列服务基线快照，用于 `tear_down` 校验资源零泄漏。
    initial: super::Baseline,
}

impl ::ktest::Fixture for ServiceFixture {
    /// 初始化测试用例夹具：迁移至 CPU0、采集资源基线、按需预分配 OOM 压力表并注册全局 [`CURRENT`] 场景指针。
    fn set_up(context: &mut ::ktest::TestContext<'_>, _: &()) -> Result<Self, ::ktest::TestError> {
        sched::test_migrate_to(0);
        let initial = super::baseline();
        let mut probes = alloc::vec::Vec::new();
        probes
            .try_reserve_exact(16)
            .map_err(|_| ::ktest::TestError("NoMemory: operation owners"))?;
        let mut owners = alloc::vec::Vec::new();
        owners
            .try_reserve_exact(4)
            .map_err(|_| ::ktest::TestError("NoMemory: source owners"))?;

        let pressure_capacity = if context.case.name.contains("real_oom_rescuer")
            || context.case.name.contains("ordinary_worker_oom")
            || context.case.name.contains("printer_real_oom")
        {
            PRESSURE_CAPACITY
        } else {
            0
        };
        let mut pressure = alloc::vec::Vec::new();
        pressure
            .try_reserve_exact(pressure_capacity)
            .map_err(|_| ::ktest::TestError("NoMemory: pressure pages"))?;
        pressure.resize_with(pressure_capacity, MaybeUninit::uninit);
        let mut objects = alloc::vec::Vec::new();
        objects
            .try_reserve_exact(pressure_capacity)
            .map_err(|_| ::ktest::TestError("NoMemory: pressure objects"))?;
        objects.resize_with(pressure_capacity, MaybeUninit::uninit);

        let state = Arc::try_new(Scenario {
            nodes: super::Nodes::new(0, 8, 2, 2)?,
            stop: AtomicBool::new(false),
            operation_drops: AtomicUsize::new(0),
            access_after_close: AtomicUsize::new(0),
            delay_source_closed: AtomicBool::new(false),
            owner_drops: AtomicUsize::new(0),
            probes: SpinLock::new(probes),
            owners: SpinLock::new(owners),
            pressure: core::cell::UnsafeCell::new(pressure),
            objects: core::cell::UnsafeCell::new(objects),
            pressure_counts: SpinLock::new((0, 0)),
        })
        .map_err(|_| ::ktest::TestError("NoMemory: execution state"))?;
        let pointer = Arc::into_raw(state.clone()).cast_mut();
        assert!(CURRENT
            .compare_exchange(
                core::ptr::null_mut(),
                pointer,
                Ordering::AcqRel,
                Ordering::Acquire
            )
            .is_ok());
        Ok(Self { state, initial })
    }

    /// 静默化（Quiesce）阶段：停止所有业务生产源、取消测试停点、放行门闸、归还残余 OOM 内存压力，并同步关闭所有注册的队列与工作项。
    fn quiesce(&mut self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        self.state.stop_sources();
        let counts = core::mem::take(&mut *self.state.pressure_counts.lock());
        release_owned_pressure(counts.0, counts.1);
        let count = self.state.owners.lock().len();
        for index in 0..count {
            let owner = self.state.owners.lock()[index].clone();
            if owner.probe.stops.load(Ordering::Acquire) == 0 {
                owner.close();
            }
        }
        self.state.nodes.stop()?;
        Ok(())
    }

    /// 拆卸（Tear Down）阶段：在测试框架回收所有子任务后，清空探测槽位、校验所有异步强引用归还与对象析构计数，并与初始基线对比确认无内存或线程泄漏。
    fn tear_down(self, _: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        let mut owner = core::mem::ManuallyDrop::new(self);
        let count = owner.state.probes.lock().len();
        for index in 0..count {
            let probe = owner.state.probes.lock()[index].clone();
            probe.clear();
            if probe.active.load(Ordering::Acquire) != 0 {
                return Err(::ktest::TestError("work callback remains active"));
            }
        }

        let source_count = owner.state.owners.lock().len();
        let sources = core::mem::take(&mut *owner.state.owners.lock());
        drop(sources);
        if owner.state.owner_drops.load(Ordering::Acquire) != source_count {
            return Err(::ktest::TestError("source owner drop count mismatch"));
        }
        if *owner.state.pressure_counts.lock() != (0, 0) {
            return Err(::ktest::TestError("pressure ownership did not return"));
        }

        owner.state.nodes.tails()?;
        let probes = core::mem::take(&mut *owner.state.probes.lock());
        drop(probes);
        if owner.state.operation_drops.load(Ordering::Acquire) != count {
            return Err(::ktest::TestError("operation drop count mismatch"));
        }

        let initial = owner.initial;
        let pointer = CURRENT.swap(core::ptr::null_mut(), Ordering::AcqRel);
        assert_eq!(pointer, Arc::as_ptr(&owner.state).cast_mut());
        // SAFETY: `pointer` 对应 `set_up` 中 `Arc::into_raw` 产生的唯一全局登记引用；此时所有工作项与子任务已彻底停止且无并发访问，恢复为 `Arc` 释放并手动析构 `owner`。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 套件级基线下的夹具初始化，直接复用单用例 `set_up` 逻辑。
    fn set_up(
        context: &mut ::ktest::TestContext<'_>,
        _: &super::SuiteBaseline,
    ) -> Result<Self, ::ktest::TestError> {
        <Self as ::ktest::Fixture<()>>::set_up(context, &())
    }

    /// 套件级基线下的夹具静默化，直接复用单用例 `quiesce` 逻辑。
    fn quiesce(&mut self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::quiesce(self, context)
    }

    /// 套件级基线下的夹具拆卸，直接复用单用例 `tear_down` 逻辑。
    fn tear_down(self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::tear_down(self, context)
    }
}

impl Scenario {
    /// 幂等地设置全局停止标志，取消所有 CPU 上的工作队列与定时器测试停点，并关闭所有 [`Probe`] 的生产源、放行全部同步门闸。
    fn stop_sources(&self) {
        if self.stop.swap(true, Ordering::AcqRel) {
            return;
        }
        for cpu in 0..cpu::count() {
            for point in [
                testing::PausePoint::Started,
                testing::PausePoint::Finished,
                testing::PausePoint::BeforeWait,
            ]
            .iter()
            .copied()
            {
                testing::cancel_pause(cpu, point);
            }
        }
        delayed_testing::cancel_pause();
        let count = self.probes.lock().len();
        for index in 0..count {
            let probe = self.probes.lock()[index].clone();
            probe.source.store(false, Ordering::Release);
            probe.release.store(true, Ordering::Release);
            for gate in &probe.gates {
                gate.complete();
            }
        }
    }
}

impl ServiceFixture {
    /// 在 CPU0 上受控执行测试闭包 `action`：若发生 Panic 展开，先调用 [`Scenario::stop_sources`] 解除停点与门闸阻塞，以便外层顺利回收子任务。
    pub(super) fn fixed(
        &self,
        action: impl FnOnce() -> ::ktest::TestResult + Send + 'static,
    ) -> ::ktest::TestResult {
        super::on_zero(self.state.clone(), Scenario::stop_sources, action)
    }
}

// SAFETY: `pressure` 与 `objects` 的底层 `Vec` 在 `set_up` 后容量与缓冲区地址固定不变；
// 内存压力构造阶段由单线程独占写入，并通过 `pressure_counts` 自旋锁将已初始化前缀唯一移交给单个回收者；其余字段均为原子类型或受 `SpinLock` 保护。
unsafe impl Send for Scenario {}

// SAFETY: 对 `pressure` 与 `objects` 缓冲区的写入与消费严格通过 `pressure_counts` 自旋锁与原子标志串行化，不存在并发数据竞争；其余字段均具备线程安全同步原语。
unsafe impl Sync for Scenario {}

/// 获取当前场景中预分配物理页块暂存数组的可变首地址。
fn pressure_pages() -> *mut MaybeUninit<mm::OwnedPages> {
    let owner = scenario();
    // SAFETY: `owner.pressure` 内的 `Vec` 在 `set_up` 后不再扩容或移动，此处仅获取底层缓冲区裸指针，槽位访问由调用方按 `pressure_counts` 协议保证互斥。
    unsafe { (*owner.pressure.get()).as_mut_ptr() }
}

/// 获取当前场景中预分配 SLUB 堆对象暂存数组的可变首地址。
fn pressure_objects() -> *mut MaybeUninit<(*mut u8, Layout)> {
    let owner = scenario();
    // SAFETY: `owner.objects` 内的 `Vec` 在 `set_up` 后不再扩容或移动，此处仅获取底层缓冲区裸指针，槽位访问由调用方按 `pressure_counts` 协议保证互斥。
    unsafe { (*owner.objects.get()).as_mut_ptr() }
}

/// 创建受当前测试夹具生命周期管理的工作队列（[`super::QueueOwner`]）。
fn new_queue(config: WorkQueueConfig) -> Arc<super::QueueOwner> {
    scenario().nodes.queue(config)
}

/// 创建受当前测试夹具生命周期管理的普通工作项（`Work`）。
fn new_work(
    queue: &WorkQueue,
    cpu: usize,
    callback: fn(Pin<&Work>),
    context: usize,
) -> Pin<Arc<Work>> {
    scenario().nodes.work(queue, cpu, callback, context)
}

/// 创建受当前测试夹具生命周期管理的延迟工作项（`DelayedWork`）。
fn new_delayed(
    queue: &WorkQueue,
    cpu: usize,
    callback: fn(Pin<&DelayedWork>),
    context: usize,
) -> Pin<Arc<DelayedWork>> {
    scenario().nodes.delayed(queue, cpu, callback, context)
}

/// 供父模块不可恢复 OOM 故障测试调用的辅助入口：耗尽系统中所有物理页与 SLUB 堆对象缓存并返回 `(页块数, 堆对象数, 耗尽规格数)`。
pub(super) fn exhaust_fault() -> (usize, usize, usize) {
    let pages = exhaust_pages();
    let (objects, nulls) = exhaust_objects();
    (pages, objects, nulls)
}

#[::ktest::suite(fixture = ServiceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "work"), timeout_ms = 45000)]
pub(crate) mod cases {
    use super::*;

    /// **测试目标**：验证工作项在入队等待期（Pending）与回调睡眠运行期（Running）重复提交时的合并语义，以及同一工作项多轮串行不重入保证。
    /// **场景步骤**：首次入队后重复提交 3 次；待首轮回调阻塞后再次入队并重复提交 3 次；放行后利用 `Started` 停点验证刚清除 Pending 时的重入队行为。
    /// **预期结果**：16 次业务更新合并为 2 次串行回调执行，历史最大活跃并发度始终为 1。
    #[test]
    fn duplicates_and_business(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_duplicates();
            Ok(())
        })
    }

    /// **测试目标**：验证有序工作队列（Ordered WorkQueue）的跨 CPU 提交归一化与全域单并发 FIFO 顺序。
    /// **场景步骤**：从最后一个 CPU 向 `ordered = true` 的队列连续提交 4 个配置了不同目标 CPU 的工作项并刷新等待。
    /// **预期结果**：所有回调均被归一化到 CPU0 上严格按提交顺序（0..4）串行执行，最大并发度为 1。
    #[test]
    fn ordered_fifo(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            assert_eq!(ordered_case(), (1, 1));
            Ok(())
        })
    }

    /// **测试目标**：验证有序工作队列中自我重排任务保留原 Worker 关联，且跨域 `flush()` 不会窃取该原执行者导致死锁。
    /// **场景步骤**：有序域任务 A 在首轮回调中入队无关域任务 B 并自我重排，次轮阻塞在门闸上；任务 B 在另一 Worker 中对任务 A 调用 `flush()`。
    /// **预期结果**：任务 A 的两轮回调由同一个原 Worker 执行，任务 B 由另一个独立 Worker 执行并在 `flush()` 上阻塞，门闸放行后两者顺利完成。
    #[test]
    fn ordered_original_worker(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            ordered_association_case();
            Ok(())
        })
    }

    /// **测试目标**：验证多层阻塞工作项链触发动态 Worker 按需创建、早到唤醒通知（Early Wake）配对以及完工后的动态 Worker 自动退休回收。
    /// **场景步骤**：提交包含 3 层阻塞依赖的工作项链（第 0 层先触发唤醒通知再 `park`），等待全部阻塞后统一放行并等待服务静默。
    /// **预期结果**：动态创建至少 `DEPTH - 1` 个新 Worker，睡眠与运行计数严格配对，且完工后动态 Worker 正常退休回收。
    #[test]
    fn blocked_dependency_depth(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_deep_blocking();
            Ok(())
        })
    }

    /// **测试目标**：验证普通队列中自我重排任务的原 Worker 关联在同域并发配额占满时的保护机制，以及在登记就绪队列前/后取消关联任务时的自动退休行为。
    /// **场景步骤**：CPU1 控制线程将原 Worker 拦截在 `Finished` 停点并占满同域配额，随后分别测试跨域 `flush()`、阻塞后 `cancel_sync()` 以及 `BeforeWait` 停点处 `cancel_sync()`。
    /// **预期结果**：原 Worker 不会被无关域窃取；当其关联的唯一工作项被取消后，无需提交新工作项即可自行退出并被 Manager 回收。
    #[test]
    #[ktest(min_cpus = 2)]
    #[case("flush", AssociationAction::Flush)]
    #[case("cancel_registered", AssociationAction::CancelBlocked)]
    #[case("cancel_before_register", AssociationAction::CancelBeforeWait)]
    fn original_worker_association(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        action: AssociationAction,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || association_case(action))
    }

    /// **测试目标**：验证当协调控制子线程在持有远核 Worker 停点期间发生 Panic 时，夹具能够安全解除停点、完成全部清理并保留子线程失败状态。
    /// **场景步骤**：CPU1 控制线程武装 CPU0 的 `Finished` 停点并等待其命中最深位置后触发受控 Panic。
    /// **预期结果**：`Coordinator` 与夹具 `quiesce` 自动取消停点并放行门闸，Worker 正常退出，测试以预期的子线程失败结束而非系统死锁。
    #[test]
    #[ktest(
        min_cpus = 2,
        ignore = "controlled child panic must remain Fail after complete cleanup"
    )]
    fn association_controller_failure(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| association_case(AssociationAction::ControllerFailure))
    }

    /// **测试目标**：验证带 `reclaim = true` 的工作队列在物理页与全部 SLUB 堆缓存耗尽（OOM）导致普通 Worker 创建失败时，能通过预分配的 Rescuer 线程完成内存回收。
    /// **场景步骤**：先阻塞唯一的常驻普通 Worker，再耗尽所有物理页与 12 个 SLUB 规格对象，随后入队回收工作项。
    /// **预期结果**：Manager 创建新 Worker 失败后唤醒 Rescuer 线程执行回收回调释放内存，随后物理页与全部 SLUB 规格恢复可分配状态。
    #[test]
    fn real_oom_rescuer(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_reclaim();
            Ok(())
        })
    }

    /// **测试目标**：验证无 Rescuer 的普通工作队列在 OOM 创建失败后的冷却等待、外部释放内存后的无新事件自动恢复，以及冷却期内取消工作项后的停止重试语义。
    /// **场景步骤**：阻塞常驻 Worker 并耗尽内存，提交新工作项触发创建失败进入冷却窗口；分别测试保持排队等待独立线程释放内存，或在冷却期内调用 `cancel_sync()`。
    /// **预期结果**：冷却期内 Manager 进入 `Blocked` 状态而不自旋；未取消时外部释放内存后自动创建新 Worker 执行回调，已取消时不再创建新 Worker。
    #[test]
    #[case("recover_without_queue", false)]
    #[case("cancel_cooldown", true)]
    fn ordinary_worker_oom(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        cancel: bool,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || {
            let (failures, callbacks) = ordinary_recovery_case(cancel);
            assert!(1 <= failures);
            assert_eq!(callbacks, usize::from(!cancel));
            Ok(())
        })
    }

    /// **测试目标**：验证 `WorkQueue::flush()` 的批次边界语义，确保其只等待调用时刻已存在的旧工作项，不被后续新入队的未来工作项阻塞。
    /// **场景步骤**：让旧工作项 0 阻塞、旧工作项 1 完成，启动 `flush()` 线程使其因工作项 0 阻塞；期间提交未来工作项 2..6 并使其保持排队或运行状态，最后放行工作项 0。
    /// **预期结果**：工作项 0 完成后 `flush()` 立即返回，此时未来工作项 2..6 仍处于活跃或排队状态。
    #[test]
    fn finite_flush_prefix(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_flush_boundary();
            Ok(())
        })
    }

    /// **测试目标**：验证单个 `DelayedWork` 的延迟入队、到期时间修改（`modify`）、到期移交执行、同步取消（`cancel_sync`）及关闭后的引用计数完整性。
    /// **场景步骤**：对同一 `DelayedWork` 依次执行 1 秒入队、修改为 10ms 并刷新、再次 10ms 入队等待完成、1 秒入队后立即同步取消，最后关闭并睡眠观测。
    /// **预期结果**：两次到期移交回调准确执行，取消与关闭后状态清零、异步强引用全部归还，且关闭后无任何迟到回调访问。
    #[test]
    fn delayed_transfer_lifetime(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_delayed();
            Ok(())
        })
    }

    /// **测试目标**：验证多核环境下 `DelayedWork` 在定时器到期移交前（`BeforeTransfer`）与移交后（`AfterTransfer`）两个停点窗口内，调用 `cancel_sync()` 必须阻塞等待移交尾部完成。
    /// **场景步骤**：将绑定 CPU1 的 `DelayedWork` 配置在指定移交停点暂停，待其命中停点后唤醒 CPU0 线程执行 `cancel_sync()`，验证阻塞状态后释放停点。
    /// **预期结果**：`cancel_sync()` 线程在停点释放前确实进入 `Blocked` 状态，释放停点后正常返回且无异步引用泄漏。
    #[test]
    #[ktest(min_cpus = 2)]
    #[case("before_transfer", delayed_testing::PausePoint::BeforeTransfer)]
    #[case("after_transfer", delayed_testing::PausePoint::AfterTransfer)]
    fn delayed_cancel_real_tail(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
        point: delayed_testing::PausePoint,
    ) -> ::ktest::TestResult {
        fixture.fixed(move || {
            let queue = new_queue(WorkQueueConfig {
                max_active: 4,
                ordered: false,
                reclaim: false,
            });
            let probe = Probe::new();
            let work = new_delayed(
                &queue,
                1,
                delayed_callback,
                &probe.entries[0] as *const Entry as usize,
            );
            assert_eq!(paused_delayed_stop(&work, &probe, point), (1, 1));
            scenario().nodes.close_delayed(work.as_ref());
            scenario()
                .delay_source_closed
                .store(true, Ordering::Release);
            queue.close_sync();
            assert_eq!(unique_delayed(&work), 0);
            probe.clear();
            Ok(())
        })
    }

    /// **测试目标**：验证当工作队列先行同步关闭（`close_sync`）后，已入队的 `DelayedWork` 定时器到期移交被拒绝时，能够自行清除 Pending 状态并归还异步强引用。
    /// **场景步骤**：入队 `DelayedWork` 使其处于定时器等待状态，先调用 `queue.close_sync()` 关闭目标队列，再放行或等待定时器到期，在不调用 `DelayedWork` 取消/关闭前检查其状态与引用计数。
    /// **预期结果**：到期移交被已关闭队列拒绝后，`DelayedWork` 自动清除 `timer_pending` 标志并归还全部异步强引用，不产生孤儿引用。
    #[test]
    fn closed_domain_due_transfer(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_rejected_transfer();
            Ok(())
        })
    }

    /// **测试目标**：验证工作项在自我重排两轮并同步取消后可正常复用，且顶层所有者（`SourceOwner`）显式关闭后永久拒绝再次入队。
    /// **场景步骤**：启动工作项完成两轮自我重排（共 3 次回调）后调用 `cancel_sync()`，随后再次入队复用 1 次并刷新，最后调用 `owner.close()` 并尝试入队。
    /// **预期结果**：累计执行 4 次回调且最大并发度为 1；显式关闭后状态标记为 `closed`，再次入队返回 `Err(QueueError::Closed)`，`Drop` 时无需重复停止。
    #[test]
    fn explicit_owner_close(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_owner();
            Ok(())
        })
    }

    /// **测试目标**：验证工作项在回调内不睡眠地持续自我重排（Self-requeue）时，Worker 仍会在回调边界响应调度请求，使同 CPU 上的普通线程获得执行机会。
    /// **场景步骤**：在 CPU0 上启动持续自我重排的工作项，首轮回调唤醒同样绑定在 CPU0 上的普通对端线程，由该对端线程将 `source` 标志置为 `false`。
    /// **预期结果**：同核对端线程成功获得 CPU 并关闭生产源，工作项完成最后一轮确认回调后正常收敛退出。
    #[test]
    fn continuous_requeue_peer(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(test_continuous_peer_progress)
    }

    /// **测试目标**：验证普通 Worker 在完成当前回调并在边界响应重调度让出后，恢复执行时能直接认领队列中未关联的后继工作项，不触发多余的动态 Worker 分配。
    /// **场景步骤**：工作项 0 在回调中入队未关联的工作项 1 并向本核发送 `KICK_RESCHEDULE`，等待工作项 1 执行完毕后对比工作池的累计创建计数 `created`。
    /// **预期结果**：两个工作项均完成执行，且工作池的 Worker 创建总数保持不变。
    #[test]
    fn callback_handoff_reuses_worker(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(|| {
            test_callback_handoff();
            Ok(())
        })
    }
}
