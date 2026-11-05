//! 以真实任务、工作域、定时器与唯一 MM 验证工作进度和一次停止责任。

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

const SLOTS: usize = 8;
const DEPTH: usize = 3;

const PRESSURE_CAPACITY: usize = 65536;

struct Entry {
    probe: usize,
    index: usize,
}

struct Probe {
    entries: [Entry; SLOTS],
    works: [SpinLock<Option<Pin<Arc<Work>>>>; SLOTS],
    tasks: [SpinLock<Option<Arc<Task>>>; SLOTS],
    gates: [Completion; SLOTS],
    counts: [AtomicUsize; SLOTS],
    active: AtomicUsize,
    maximum: AtomicUsize,
    submitted: AtomicUsize,
    consumed: AtomicUsize,
    callbacks: AtomicUsize,
    release: AtomicBool,
    source: AtomicBool,
    started: AtomicBool,
    done: AtomicBool,
    pages: AtomicUsize,
    objects: AtomicUsize,
    rescuer: AtomicUsize,
    ordinary: AtomicUsize,
    stops: AtomicUsize,
}

impl Probe {
    /// 全部观察、完成源和稳定上下文在发表任何异步票前一次预备。
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
            // SAFETY: raw 来自尚未发表的唯一 Arc::into_raw；没有导出共享借用，
            // 只在固定分配内初始化 Entry 的自引用，之后所有访问均为共享访问。
            unsafe {
                let entry = core::ptr::addr_of_mut!((*raw.cast_mut()).entries)
                    .cast::<Entry>()
                    .add(index);
                core::ptr::addr_of_mut!((*entry).probe).write(raw as usize);
            }
        }
        // SAFETY: 使用上面唯一且原样保存的 into_raw 指针恢复构造拥有，期间未释放或增加票。
        {
            // SAFETY: raw恢复上方唯一真实into_raw拥有；仍未发表任何回调或child。
            let owner = unsafe { Arc::from_raw(raw) };
            scenario().probes.lock().push(owner.clone());
            owner
        }
    }

    /// 创建固定函数和工作域的实际 Work，测试表只克隆现有强引用。
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

    /// 不分配地取得下一层已准备的真实工作。
    fn next_work(&self, index: usize) -> Pin<Arc<Work>> {
        self.works[index]
            .lock()
            .as_ref()
            .expect("prepared work")
            .clone()
    }

    /// 发布真实 callback Task 身份，观察锁不跨生产调度锁。
    fn publish_task(&self, index: usize) {
        let current = task::current_task();
        assert!(self.tasks[index].lock().replace(current).is_none());
    }

    /// 读取已发表 Task 的真实状态，不在观察锁内进入调度器。
    fn blocked(&self, index: usize) -> bool {
        let observed = self.tasks[index].lock().as_ref().map(Arc::clone);
        observed.is_some_and(|task| sched::test_state(&task) == State::Blocked)
    }

    /// 在所有回调与取消任务结束后移出观察拥有，最终析构均在短锁外。
    fn clear(&self) {
        for index in 0..SLOTS {
            let work = self.works[index].lock().take();
            let task = self.tasks[index].lock().take();
            drop((work, task));
        }
    }
}

impl Drop for Probe {
    /// 只观察稳定 operation 的一次析构，不在 Drop 再次同步关闭。
    fn drop(&mut self) {
        assert_eq!(self.active.load(Ordering::Acquire), 0);
        assert!(self.works.iter().all(|work| work.lock().is_none()));
        assert!(self.tasks.iter().all(|task| task.lock().is_none()));
        scenario().operation_drops.fetch_add(1, Ordering::AcqRel);
    }
}

/// 从构造时已固定的 Entry 读取共享业务上下文，借用不能超过真实 callback 的同步尾部。
///
/// # Safety
/// address 必须来自仍存活的 Probe Arc 内 Entry；调用者必须保持该 Arc 到本借用结束，
/// 且直到所有使用该 Entry 的 Task 已 join、Work/DelayedWork 已同步关闭并归还异步强票。
/// Entry 发表后不得修改；返回值不能逃逸至 owner 停止后的任何访问。
unsafe fn context<'a>(address: usize) -> (&'a Probe, usize) {
    // SAFETY: 调用者保证 Entry 及其 Probe 的真实 Arc 分配在整个借用期间存活且只读。
    let entry = unsafe { &*(address as *const Entry) };
    // SAFETY: probe 是未发表构造阶段保存的原 Arc raw 地址，实际 owner 保活且不产生独占重借。
    (unsafe { &*(entry.probe as *const Probe) }, entry.index)
}

/// 以真实硬件计数建立一次总期限，不靠重试延长或周期扫描提供生产进度。
fn raw_deadline() -> u64 {
    arch::read_counter()
        .checked_add(arch::counter_frequency().checked_mul(10).unwrap())
        .unwrap()
}

/// 有限轮询仅协调测试参与者，实际阻塞和唤醒始终经过生产等待源。
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

/// 远核 BH 停点不能睡眠；观察端只忙等已经武装的一次原子握手。
fn spin_until(mut predicate: impl FnMut() -> bool, name: &str) {
    let end = raw_deadline();
    while !predicate() {
        assert!(arch::read_counter() < end, "work pause timeout: {}", name);
        spin_loop();
    }
}

/// 真实常驻 worker、manager、timer 服务均已等待，动态 worker 已退出才允许取基线。
pub(super) fn wait_services_blocked() {
    testing::wait_services_blocked();
    super::timer::wait_services_blocked();
}

/// 只有测试 owner 留下的真实强引用才允许结束上下文寿命。
fn unique_work(work: &Pin<Arc<Work>>) {
    // SAFETY: 克隆仅用于观察控制块，不移动 Work；原 Pin 和观察期间的强拥有保持节点稳定。
    let observer = unsafe { Pin::into_inner_unchecked(work.clone()) };
    until(
        || Arc::strong_count(&observer) == 3,
        "work asynchronous reference returned",
    );
    drop(observer);
}

/// 不清理 D 的状态就观察真实异步强票归还，不能用 close/cancel 掩盖到期孤票。
fn unique_delayed(work: &Pin<Arc<DelayedWork>>) -> usize {
    // SAFETY: 克隆 Arc 仅观察控制块，不移动 D；原 Pin 与 owner 在整个观察期间保活。
    let observer = unsafe { Pin::into_inner_unchecked(work.clone()) };
    until(
        || Arc::strong_count(&observer) == 3,
        "delayed asynchronous tickets returned",
    );
    let orphan = Arc::strong_count(&observer) - 3;
    assert_eq!(orphan, 0);
    orphan
}

/// 全域有序 callback 验证实际 CPU、FIFO 顺序与未结束 callback 的唯一准入。
fn ordered_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: ordered_case 持有 Probe Arc，逐项 close、归还强票并 join 提交者后才释放。
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

/// 真实异 CPU Task 向同一 ordered 域提交，绑定规范化和全域 FIFO 都由回调断言。
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
    assert_eq!(handle.join().unwrap(), 4);
    queue.flush();
    assert_eq!(probe.consumed.load(Ordering::Acquire), 4);
    assert_eq!(probe.maximum.load(Ordering::Acquire), 1);

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

/// ordered A 先排别域 B 再排自身；第二轮真睡眠让 B 在另一执行者中同步等待 A。
fn ordered_association_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: ordered_association_case 保留 Probe Arc，全部回调完成、Work/domain 同步关闭及归票后才释放。
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

/// 单核也使用真实两个域与阻塞调度，不能用异核停点代替 ordered 自关联进度。
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

    a.as_ref().close_sync();
    b.as_ref().close_sync();
    ordered.close_sync();
    unrelated.close_sync();
    probe.clear();
    unique_work(&a);
    unique_work(&b);
    drop((a, b, ordered, unrelated, probe));
}

/// 业务计数独立于 pending；第一轮可睡，下一轮与 running 同存但绝不重入。
fn duplicate_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: test_duplicates 持有 Probe Arc 到 gate 放行、Work 同步关闭及异步强票归还。
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

/// claim 清 pending 后停点释放，实际业务再次核验同一 Work 的串行下一轮。
fn started_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: test_duplicates 在解除真实停点、同步关闭 started 并归还其强票后才释放 Probe Arc。
    let (probe, index) = unsafe { context(work.context()) };
    let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
    assert_eq!(active, 1);
    probe.maximum.fetch_max(active, Ordering::AcqRel);
    probe.counts[index].fetch_add(1, Ordering::AcqRel);
    assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// 运行前与可睡运行中重复提交都保持真实业务合并，同一Work不重入。
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

    probe.submitted.store(8, Ordering::Release);
    assert!(work.as_ref().queue().unwrap());
    let mut before_duplicates = 0;
    for _ in 0..3 {
        before_duplicates += usize::from(!work.as_ref().queue().unwrap());
    }
    assert_eq!(before_duplicates, 3);
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

    probe.gates[0].complete();
    work.as_ref().flush();
    queue.flush();
    let submitted = probe.submitted.load(Ordering::Acquire);
    let consumed = probe.consumed.load(Ordering::Acquire);
    let callbacks = probe.callbacks.load(Ordering::Acquire);
    let maximum = probe.maximum.load(Ordering::Acquire);
    assert_eq!((submitted, consumed, callbacks, maximum), (16, 16, 2, 1));

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

/// 基础执行者与同域占位者真睡眠；关联票只在第一轮 callback 内 selfqueue。
fn association_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: association_case 持有 Probe Arc，控制者 join、全部 Work/domain 关闭及归票后才释放。
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

/// 同一真实关联交错分别覆盖跨域同步、登记后取消和首次退休检查后的登记前取消。
enum AssociationAction {
    Flush,
    CancelBlocked,
    CancelBeforeWait,
    ControllerFailure,
}

/// controller展开时先放行自己武装的CPU0停点与原gate，不等待被它占住的协调CPU。
fn release_association(probe: &Probe) {
    testing::cancel_pause(0, testing::PausePoint::Finished);
    testing::cancel_pause(0, testing::PausePoint::BeforeWait);
    for index in [0, 2, 4] {
        probe.gates[index].complete();
    }
}

/// 独立 CPU1 控制者协调 CPU0 完成停点，不用测试轮询替生产 worker 提供进度。
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

/// 三层真正阻塞依赖交给动态 worker，叶子仅等待测试解除后从最深层逐级唤醒。
fn chain_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: test_deep_blocking 保持 Probe Arc，所有依赖放行并逐项 close、归票后才释放上下文。
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

/// 三层依赖同时Blocked时观察真实创建、早通知配对和动态退休。
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
    assert!(works[0].as_ref().queue().unwrap());
    until(
        || (0..DEPTH).all(|index| probe.blocked(index)) && probe.tasks[DEPTH].lock().is_some(),
        "three blocked worker dependencies",
    );
    let blocked = (0..DEPTH).filter(|&index| probe.blocked(index)).count();
    let creators = testing::pool_stats(0).created - before.created;
    assert_eq!(blocked, DEPTH);
    assert!(DEPTH - 1 <= creators);

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

/// 用现有唯一页入口从大块到单页真正摘尽 RAM；容量耗尽是测试错误而非 OOM。
fn exhaust_pages() -> usize {
    mm::drain_all_pages();
    let mut count = 0;
    for order in (0..usize::BITS as usize).rev() {
        loop {
            assert!(count < PRESSURE_CAPACITY, "work pressure page storage full");
            let Some(pages) = mm::alloc_pages(order, false) else {
                break;
            };
            // SAFETY: 压力构造仅由 CPU0 协调者执行；count 未初始化且在界内，移动保存唯一页拥有权。
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

/// 页已摘尽后继续真实消费每个生产 SLUB 类的残余对象，直到 GlobalAlloc 返回 null。
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
            // SAFETY: layout 有效且 MM 已初始化；成功分配的唯一归还权立即保存到压力表。
            let pointer = unsafe { alloc(layout) };
            if pointer.is_null() {
                nulls += 1;
                break;
            }
            // SAFETY: 协调者独占尚未初始化的 count 槽，保存原 live 指针及原 Layout。
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

/// 已发表压力前缀只交给一个回收 callback 或独立 Task，归还时不持池锁。
fn release_pressure(page_count: usize, object_count: usize) {
    let actual = core::mem::take(&mut *scenario().pressure_counts.lock());
    if actual == (0, 0) && scenario().stop.load(Ordering::Acquire) {
        return;
    }
    assert_eq!(actual, (page_count, object_count));
    release_owned_pressure(page_count, object_count);
}

/// 只有counts原锁移出的唯一已初始化前缀才能消费原页和原Layout。
fn release_owned_pressure(page_count: usize, object_count: usize) {
    for index in (0..object_count).rev() {
        // SAFETY: 两个完整前缀由协调者初始化后 Release 发表给唯一回收者；
        // 协调者已结束压力表访问，每槽在此移动一次，未发生过归还。
        let (pointer, layout) = unsafe {
            pressure_objects()
                .cast::<MaybeUninit<(*mut u8, Layout)>>()
                .add(index)
                .read()
                .assume_init()
        };
        // SAFETY: 刚取得原分配的唯一归还权，pointer/layout 完全匹配，压力对象没有其他访问者。
        unsafe { dealloc(pointer, layout) };
    }

    for index in (0..page_count).rev() {
        // SAFETY: 已初始化原前缀的唯一页拥有者只在此移动消费，回收者外没有并发压力表访问。
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

/// 保持基础普通 worker 真正 Blocked，不能由它执行下一项受控解压工作。
fn pressure_blocker(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: 场景 owner 在解除 blocker、同步关闭 Work、join 独立任务并归票前保留 Probe Arc。
    let (probe, _) = unsafe { context(work.context()) };
    assert!(!testing::is_rescuer());
    probe.publish_task(0);
    probe.gates[0].wait();
}

/// 唯一受控回收 work 核验实际 worker 身份，再归还先前准备的真实压力资源。
fn reclaim_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: test_reclaim 在真实压力释放回调完成、rescuer join 且 recovery 归票前保留 Probe Arc。
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

/// 预建rescuer必须在普通Worker真实创建OOM后执行，原页和全部heap类放压后恢复。
fn test_reclaim() {
    let queue = new_queue(WorkQueueConfig {
        max_active: 4,
        ordered: false,
        reclaim: true,
    });
    let probe = Probe::new();
    let blocker = probe.work(&queue, 0, 0, pressure_blocker);
    let recovery = probe.work(&queue, 1, 0, reclaim_callback);
    assert!(blocker.as_ref().queue().unwrap());
    until(|| probe.blocked(0), "ordinary worker blocked before OOM");
    let before = testing::pool_stats(0);
    assert_eq!(before.workers, 1);
    assert_eq!(
        before.idle, 0,
        "ordinary idle worker could bypass OOM rescue"
    );

    mm::shrink_all();
    let page_count = exhaust_pages();
    let (objects, _heap_null) = exhaust_objects();
    let page_null = usize::from(mm::alloc_pages(0, false).is_none());
    assert_eq!(page_null, 1);
    probe.pages.store(page_count, Ordering::Release);
    probe.objects.store(objects, Ordering::Release);
    assert!(recovery.as_ref().queue().unwrap());
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

    let recovered_page = mm::alloc_pages(0, false).expect("page allocation did not recover");
    drop(recovered_page);
    let mut heap_recovered = 0;
    for size in mm::SLUB_SIZES {
        let layout = Layout::from_size_align(size, 8).unwrap();
        // SAFETY: 有效布局由唯一协调者使用，刚释放的压力应让每个生产类重新成功分配。
        let pointer = unsafe { alloc(layout) };
        assert!(!pointer.is_null(), "heap class did not recover");
        heap_recovered += usize::from(!pointer.is_null());
        // SAFETY: pointer 是本轮同 layout 成功分配的原地址，唯一协调者未交给任何其他访问者。
        unsafe { dealloc(pointer, layout) };
    }
    let recovered = usize::from(heap_recovered == mm::SLUB_SIZES.len());
    assert_eq!(recovered, 1);

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

/// 所有普通恢复阶段共用一个硬件计数期限，协调轮询不产生工作池事件。
fn ordinary_until(end: u64, mut predicate: impl FnMut() -> bool, name: &str) {
    while !predicate() {
        assert!(arch::read_counter() < end, "ordinary OOM timeout: {}", name);
        sched::yield_now();
    }
}

/// 跨越一秒创建冷却并观察真实 Blocked，持续压力下失败增量必须有界，撤票后必须为零。
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

/// 预建普通 Task 独占压力表的归还权，放压后继续真睡眠，避免离栈释放代替恢复条件。
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
        // SAFETY: 有效生产类布局，唯一普通 Task 取得并立即归还本轮分配，没有导出访问者。
        let pointer = unsafe { alloc(layout) };
        assert!(
            !pointer.is_null(),
            "independent release did not recover heap class"
        );
        // SAFETY: pointer/layout 是刚成功取得的原分配，唯一拥有尚未归还。
        unsafe { dealloc(pointer, layout) };
    }
    // 此 Task 固定 CPU0，归还与验证期间没有调度点；发表恢复后才 park，让 CPU0 manager 继续。
    probe.release.store(true, Ordering::Release);
    probe.gates[3].wait();
    20
}

/// B 只能由新普通执行者在独立放压后执行，业务完成随后唤醒始终等待的基础 A。
fn ordinary_recovery_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: ordinary_recovery_case 保活 Probe 到两项 Work 同步关闭、独立 Task join 及观察归票以后。
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

/// 普通域真实失败后保留 B 等独立恢复，或冷却中撤唯一 B；全部资源在 OOM 前准备。
fn ordinary_recovery_case(cancel: bool) -> (usize, usize) {
    let queue = new_queue(WorkQueueConfig {
        max_active: 2,
        ordered: false,
        reclaim: false,
    });
    let probe = Probe::new();
    let blocker = probe.work(&queue, 0, 0, pressure_blocker);
    let pending = probe.work(&queue, 1, 0, ordinary_recovery_callback);
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
    if cancel {
        assert!(pending.as_ref().cancel_sync());
        ordinary_creation_window(&probe, pending.as_ref(), false, failed, 0, end);
    } else {
        ordinary_creation_window(&probe, pending.as_ref(), true, failed, 2, end);
    }
    let failures = testing::pool_stats(0).create_failures - before.create_failures;
    assert!(1 <= failures);

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

/// 较早 work 在指定真实 CPU 阻塞，较晚项完成后，未来项继续独立占据 pending/running。
fn boundary_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: test_flush_boundary 先切生产源、解除全部 gate、逐项 close 并归票后才释放 Probe Arc。
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

/// 等待固定旧颜色的每张票，有限未来队列不能延长已切定的flush边界。
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

    let flush_probe = probe.clone();
    let queue_address = Arc::as_ptr(&queue) as usize;
    let flusher = crate::ktest::spawn_result_on(0, move |_| {
        Ok({
            let probe = flush_probe;
            probe.publish_task(7);
            probe.started.store(true, Ordering::Release);
            // SAFETY: fixture在catch外持有同一个固定Arc owner，原scope join之前不释放；此借用仅同步flush。
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

/// 业务回调只能在绑定 worker 上执行；关闭哨兵先于 context 解引用，记录真实迟到访问。
fn delayed_callback(work: Pin<&DelayedWork>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    if scenario().delay_source_closed.load(Ordering::Acquire) {
        scenario().access_after_close.fetch_add(1, Ordering::AcqRel);
        return;
    }
    // SAFETY: test_delayed 对每个 D 完成显式 close 与真实异步归票后才释放同一个 Probe Arc。
    let (probe, _) = unsafe { context(work.context()) };
    assert_eq!(cpu::current().id, work.cpu());
    assert_eq!(cpu::current().bh_depth.load(Ordering::Relaxed), 0);
    let active = probe.active.fetch_add(1, Ordering::AcqRel) + 1;
    probe.maximum.fetch_max(active, Ordering::AcqRel);
    assert_eq!(active, 1);
    probe.callbacks.fetch_add(1, Ordering::Release);
    assert_eq!(probe.active.fetch_sub(1, Ordering::AcqRel), 1);
}

/// 远 CPU 的真实 timer 移交停点使 cancel Task 必须 Blocked，不能提前结束 operation 存储寿命。
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

/// 域关闭不能代关 timer：真实到期被拒后须自行撤 pending 并归还 D 票，之后才允许清理 D。
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
        // 正式睡眠只让真实粗 timer 有执行机会；完成仍由下方 pending 与真实强票尾部共同证明。
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
    // 真实服务重新 Blocked 证明粗 timer 的生产尾部已完成，不能用 D 清理操作替它收尾。
    super::timer::wait_services_blocked();
    assert_eq!(probe.callbacks.load(Ordering::Acquire), callbacks);
    // 上述断言全部先于任何 D 同步操作；旧拒绝分支遗留的 pending/强票会在此之前失败。
    scenario().nodes.close_delayed(work.as_ref());
    assert!(delayed_testing::snapshot(work.as_ref()).closed);
    orphan
}

/// 同一个D的真实延迟、修改、移交和取消必须归还每份拥有票，关闭后无迟到context访问。
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

struct SourceOwner {
    probe: Arc<Probe>,
    work: Pin<Arc<Work>>,
}

impl SourceOwner {
    /// 唯一 owner 先切断真实生产源，再显式一次同步关闭，普通 Drop 不重复停止。
    fn close(&self) {
        assert_eq!(self.probe.stops.load(Ordering::Acquire), 0);
        self.probe.source.store(false, Ordering::Release);
        self.work.as_ref().close_sync();
        assert_eq!(self.probe.stops.fetch_add(1, Ordering::AcqRel), 0);
    }
}

impl Drop for SourceOwner {
    /// 只核已经显式关闭的责任并计一次析构，不等待或再次调用 stop。
    fn drop(&mut self) {
        assert_eq!(self.probe.stops.load(Ordering::Acquire), 1);
        assert!(testing::snapshot(self.work.as_ref()).closed);
        assert_eq!(self.probe.active.load(Ordering::Acquire), 0);
        scenario().owner_drops.fetch_add(1, Ordering::AcqRel);
    }
}

/// 两次 self-requeue 来自实际业务 callback，running 与下一轮票共存而不并行。
fn requeue_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: SourceOwner 同持 Probe Arc 与 Work；close 先切生产源，同步尾部与强票归还后才析构。
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

/// cancel后同Work实际复用，owner显式关闭一次，Drop不再等待或访问退役存储。
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

/// 持续投递后继票，只有同 CPU 的普通 peer 停止业务源后才确认结束；回调不睡眠或让出。
fn continuous_requeue_callback(work: Pin<&Work>) {
    let owner = scenario();
    if owner.stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: catch 外 fixture 持有 Probe，Entry 地址固定；Work 关闭与 peer join 前不清除拥有，
    // 本借用仅用于当前回调，不逃逸到节点或控制者退出以后。
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

/// 同 CPU 的普通 peer 必须从连续回调中获得执行权，并等生产源停止被真实回调确认。
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

/// 首轮只投递未关联后继并请求真实调度；回调不阻塞，让出发生在生产票尾。
fn handoff_callback(work: Pin<&Work>) {
    if scenario().stop.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: fixture 保活固定 Probe 与 Entry，两个 Work 同步关闭前不清除拥有；
    // 借用只用于当前回调，不存入其它异步对象。
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

/// 已完成的普通 worker 让出后仍能领取未关联票，不为这次交接额外分配 Task 或栈。
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

/// 原closed-domain移交必须自行撤pending和归票，任何D同步清理都晚于这些断言。
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

struct Scenario {
    nodes: super::Nodes,
    stop: AtomicBool,
    operation_drops: AtomicUsize,
    access_after_close: AtomicUsize,
    delay_source_closed: AtomicBool,
    owner_drops: AtomicUsize,
    probes: SpinLock<alloc::vec::Vec<Arc<Probe>>>,
    owners: SpinLock<alloc::vec::Vec<Arc<SourceOwner>>>,
    pressure: core::cell::UnsafeCell<alloc::vec::Vec<MaybeUninit<mm::OwnedPages>>>,
    objects: core::cell::UnsafeCell<alloc::vec::Vec<MaybeUninit<(*mut u8, Layout)>>>,
    pressure_counts: SpinLock<(usize, usize)>,
}

static CURRENT: core::sync::atomic::AtomicPtr<Scenario> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// 只克隆fixture已保存的真实into_raw拥有；所有回调关闭且child原join后才撤登记。
fn scenario() -> Arc<Scenario> {
    let pointer = CURRENT.load(Ordering::Acquire);
    assert!(!pointer.is_null(), "execution fixture absent");
    // SAFETY: 同VM顺序case，fixture在catch外持Arc与登记票；quiesce先停所有源，原scope join后才撤指针。
    unsafe {
        Arc::increment_strong_count(pointer);
        Arc::from_raw(pointer)
    }
}

/// 每个工作或fatal场景独立拥有原节点、操作上下文和真实压力页对象。
pub(super) struct ServiceFixture {
    state: Arc<Scenario>,
    initial: super::Baseline,
}

impl ::ktest::Fixture for ServiceFixture {
    /// 每个case从真实常驻基线重新准备独立计数、等待源及原节点owner表。
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

    /// 先合作停止真实生产者并释放停点/gate，再同步停止原节点；不在失败时收栈。
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

    /// runner已join每个原Task；尾票归还以后才释放context/owner并比较完整两账。
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
        // SAFETY: 此指针对应setup保存的唯一登记票；所有异步源停止、原scope已join且无回调借用。
        unsafe {
            drop(Arc::from_raw(pointer));
            core::mem::ManuallyDrop::drop(&mut owner);
        }
        super::reclaimed(initial)
    }
}

impl ::ktest::Fixture<super::SuiteBaseline> for ServiceFixture {
    /// 普通工作suite沿用同一具体拥有；两个不可恢复fatal仍使用Unit资源。
    fn set_up(
        context: &mut ::ktest::TestContext<'_>,
        _: &super::SuiteBaseline,
    ) -> Result<Self, ::ktest::TestError> {
        <Self as ::ktest::Fixture<()>>::set_up(context, &())
    }

    /// suite基线不产生工作票，原压力、gate和节点责任只停止一次。
    fn quiesce(&mut self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::quiesce(self, context)
    }

    /// case原join及强票清理不变，最后由suite资源检查跨例账务。
    fn tear_down(self, context: &mut ::ktest::TestContext<'_>) -> ::ktest::TestResult {
        <Self as ::ktest::Fixture<()>>::tear_down(self, context)
    }
}

impl Scenario {
    /// 动作已退栈后先解除各CPU真实停点，再一次关闭业务生产条件并放行原gate。
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
    /// 工作/故障动作只捕获独立Scenario拥有；fixed协调者失败先解除gate，Any父才可执行真正关闭。
    pub(super) fn fixed(
        &self,
        action: impl FnOnce() -> ::ktest::TestResult + Send + 'static,
    ) -> ::ktest::TestResult {
        super::on_zero(self.state.clone(), Scenario::stop_sources, action)
    }
}

// SAFETY: pressure/objects固定Vec不再扩容，body在发表回收前独占写入；counts短锁把每个已初始化前缀只移给一个回收者。其他可变字段均为原子或SpinLock。
unsafe impl Send for Scenario {}

// SAFETY: 同一原子发表和counts唯一消费协议排除压力槽并发访问；quiesce保活State直到原scope join和真实节点尾部。
unsafe impl Sync for Scenario {}

/// 只取固定Vec的存储地址；调用方必须已经拥有counts中对应的未初始化或已初始化槽。
fn pressure_pages() -> *mut MaybeUninit<mm::OwnedPages> {
    let owner = scenario();
    // SAFETY: Vec长度/容量setup后不变，返回存储仍由catch外fixture保活，不产生元素借用。
    unsafe { (*owner.pressure.get()).as_mut_ptr() }
}

/// 原对象指针和Layout只写入未初始化槽，唯一回收者按原前缀移动消费。
fn pressure_objects() -> *mut MaybeUninit<(*mut u8, Layout)> {
    let owner = scenario();
    // SAFETY: 固定Vec不再移动/扩容；counts短锁和生产前发表约束由读写者遵守。
    unsafe { (*owner.objects.get()).as_mut_ptr() }
}

/// 域owner在正文捕获之外保存，真实rescuer仍由原close_sync负责join。
fn new_queue(config: WorkQueueConfig) -> Arc<super::QueueOwner> {
    scenario().nodes.queue(config)
}

/// 原Work在任何queue之前存入typed fixture owner表。
fn new_work(
    queue: &WorkQueue,
    cpu: usize,
    callback: fn(Pin<&Work>),
    context: usize,
) -> Pin<Arc<Work>> {
    scenario().nodes.work(queue, cpu, callback, context)
}

/// 原DelayedWork的稳定拥有先于真实timer移交发表。
fn new_delayed(
    queue: &WorkQueue,
    cpu: usize,
    callback: fn(Pin<&DelayedWork>),
    context: usize,
) -> Pin<Arc<DelayedWork>> {
    scenario().nodes.delayed(queue, cpu, callback, context)
}

/// 独立不可恢复OOM场景复用同一个fixture拥有的真实压力槽，原allocation handler负责fatal。
pub(super) fn exhaust_fault() -> (usize, usize, usize) {
    let pages = exhaust_pages();
    let (objects, nulls) = exhaust_objects();
    (pages, objects, nulls)
}

#[::ktest::suite(fixture = ServiceFixture, suite_resources = crate::ktest::execution_services::SuiteBaseline, context = "Thread", tags("execution", "work"), timeout_ms = 45000)]
pub(crate) mod cases {
    use super::*;

    /// pending前后重复提交、真实业务16次合并与同Work下一轮保持唯一Worker。
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

    /// 跨CPU提交仍规范到CPU0，严格FIFO并保持全域额度一。
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

    /// ordered自关联后续必须保留原执行者，别域合法flush由另一Worker真正阻塞。
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

    /// 三层真Blocked依赖迫使动态Worker创建，早唤醒反馈配对且原manager真实join退休。
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

    /// 原Worker在不同域flush与登记前/后取消窗口均不可被借走，撤末票不需新work才能退休。
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

    /// 控制者在真实worker停点内失败须正常quiesce/join并保child Fail，不能退化成worker fatal。
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

    /// 真页和十二heap类耗尽、Worker创建失败后只有预建rescuer能放压，并恢复所有类。
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

    /// 普通域真实OOM冷却保持原B需求；独立Task放压继续Blocked且无新增queue事件，或撤票后停止重试。
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

    /// 更早票未完时较晚票可完成，固定旧颜色flush不等待边界之后仍running/queued的未来票。
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

    /// 同一个DelayedWork真实修改、timer移交及前后取消尾部，没有孤票或存储结束后访问。
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

    /// 每个移交前/后窗口重新准备原域、D和context，同步停止必须在真实timer尾部之前Blocked。
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

    /// close域以后到期拒绝在任何D停止调用之前自行归票，覆盖旧孤pending/强引用根因。
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

    /// selfqueue真实两轮、cancel后复用、owner一次stop/drop及close永久拒绝。
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

    /// 连续 self-requeue 不能阻止同 CPU 的普通 peer 关闭真实生产源。
    #[test]
    fn continuous_requeue_peer(
        _: &mut ::ktest::TestContext<'_>,
        fixture: &mut ServiceFixture,
    ) -> ::ktest::TestResult {
        fixture.fixed(test_continuous_peer_progress)
    }

    /// 未关联后继在票尾调度后由现有 worker 领取，不增加执行者或栈分配。
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
