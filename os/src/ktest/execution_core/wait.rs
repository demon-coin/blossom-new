//! 每例独立的源队列、原Task观察、完成token、互斥占有与真实OOM压力。

use crate::ktest::execution_core::{baseline, reclaimed, Baseline};
use crate::ktest::execution_services::SuiteBaseline;
use crate::ktest::{spawn_result, spawn_result_on, ChildHandle, ChildToken};
use crate::mm::SLUB_SIZES as CLASSES;
use crate::sync::{Completion, SleepMutex, SpinLock};
use crate::task::{self, State, Task};
use crate::wait::queue::{WaitNode, WaitQueue};
use crate::wait::{Waiter, Waker};
use crate::{cpu, mm, sched};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::alloc::{alloc, dealloc};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::alloc::Layout;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

const TASKS: usize = 4;
const ROUNDS: usize = 4;
const PRESSURE_CAPACITY: usize = 65536;

/// 局部协调失败交给主体捕获；具体fixture仍保所有异步源与拥有者。
fn until(predicate: impl FnMut() -> bool, reason: &'static str) {
    crate::ktest::execution_core::until(predicate, reason).expect(reason);
}

/// 有限观察先给主体失败留下具体quiesce机会；随后等待原JoinState，不复制返回值。
fn join(handle: &ChildHandle) -> Result<i32, TestError> {
    crate::ktest::execution_core::until(
        || handle.try_join().is_some(),
        "wait child did not reach original off-stack completion",
    )?;
    handle.join()
}

struct WakeOnReturn(Arc<Control>);
impl Drop for WakeOnReturn {
    /// 远端入口发生普通失败也通知原父Waiter，使父能走具体quiesce与原join账本。
    fn drop(&mut self) {
        let waker = self.0.slots[0].waker.lock().as_ref().cloned();
        if let Some(waker) = waker {
            waker.notify();
        }
    }
}

struct Slot {
    task: SpinLock<Option<Arc<Task>>>,
    waker: SpinLock<Option<Waker>>,
    phase: AtomicUsize,
    active: AtomicUsize,
    maximum: AtomicUsize,
    switches: AtomicUsize,
}

impl Slot {
    /// 为固定数量的执行者预备观察存储，不在等待阶段分配。
    const fn new() -> Self {
        Self {
            task: SpinLock::new(None),
            waker: SpinLock::new(None),
            phase: AtomicUsize::new(0),
            active: AtomicUsize::new(0),
            maximum: AtomicUsize::new(0),
            switches: AtomicUsize::new(0),
        }
    }

    /// 在源锁外取得当前执行者的强引用，再向协调者发布只读身份。
    fn publish_task(&self) {
        let current = task::current_task();
        let previous = self.task.lock().replace(current);
        assert!(previous.is_none());
    }

    /// 克隆已经准备的控制块，只读调度状态时不保留观察锁。
    fn task(&self) -> Arc<Task> {
        Arc::clone(
            self.task
                .lock()
                .as_ref()
                .expect("worker task not published"),
        )
    }

    /// 对真实等待轮次发出通知，不持有保存 Waker 的测试锁。
    fn notify(&self) -> bool {
        let waker = self
            .waker
            .lock()
            .as_ref()
            .expect("worker waker not published")
            .clone();
        waker.notify()
    }

    /// 清空已经结束的任务与弱引用，所有最终析构均在短锁外发生。
    fn clear(&self) {
        let task = self.task.lock().take();
        let waker = self.waker.lock().take();
        drop((task, waker));
        self.phase.store(0, Ordering::Relaxed);
        self.active.store(0, Ordering::Relaxed);
        self.maximum.store(0, Ordering::Relaxed);
        self.switches.store(0, Ordering::Relaxed);
    }
}

struct Control {
    slots: [Slot; TASKS],
    queue: WaitQueue,
    drain_queue: WaitQueue,
    stop: AtomicBool,
    drain_running: AtomicBool,
    drain_initial: AtomicUsize,
    drain_rearms: AtomicUsize,
    old_waker: SpinLock<Option<Waker>>,
    payload: SleepMutex<usize>,
    start_oom: Completion,
    finish_oom: Completion,
    oom_progress: AtomicUsize,
    failed_try: AtomicUsize,
    parked: AtomicUsize,
    migrations: AtomicUsize,
    tokens: Completion,
    consumed: AtomicUsize,
}
struct HeapOwner {
    pointer: *mut u8,
    layout: Layout,
}
impl Drop for HeapOwner {
    /// 唯一压力拥有者按原指针与原布局归还，不复制指针或伪造分配失败。
    fn drop(&mut self) {
        // SAFETY: pointer为本例alloc成功的原live首址，layout原样保存，本对象不可复制且只Drop一次。
        unsafe { dealloc(self.pointer, self.layout) };
    }
}
struct Waiting {
    initial: Baseline,
    state: Arc<Control>,
    handles: Vec<ChildHandle>,
    pages: Vec<mm::OwnedPages>,
    objects: Vec<HeapOwner>,
    oom_pages: Option<usize>,
}
impl Fixture<SuiteBaseline> for Waiting {
    /// 每case全新源与计数；只有OOM例在仍有内存时预备全部压力拥有槽。
    fn set_up(context: &mut TestContext<'_>, _: &SuiteBaseline) -> Result<Self, TestError> {
        let initial = baseline()?;
        let state = Arc::try_new(Control {
            slots: core::array::from_fn(|_| Slot::new()),
            queue: WaitQueue::new(),
            drain_queue: WaitQueue::new(),
            stop: AtomicBool::new(false),
            drain_running: AtomicBool::new(true),
            drain_initial: AtomicUsize::new(0),
            drain_rearms: AtomicUsize::new(0),
            old_waker: SpinLock::new(None),
            payload: SleepMutex::new(0),
            start_oom: Completion::new(),
            finish_oom: Completion::new(),
            oom_progress: AtomicUsize::new(0),
            failed_try: AtomicUsize::new(0),
            parked: AtomicUsize::new(0),
            tokens: Completion::new(),
            migrations: AtomicUsize::new(0),
            consumed: AtomicUsize::new(0),
        })
        .map_err(|_| TestError("NoMemory: wait controls"))?;
        let mut handles = Vec::new();
        handles
            .try_reserve_exact(8)
            .map_err(|_| TestError("NoMemory: child observation slots"))?;
        let mut pages = Vec::new();
        let mut objects = Vec::new();
        if context.case.name.ends_with("::registration_under_oom") {
            pages
                .try_reserve_exact(PRESSURE_CAPACITY)
                .map_err(|_| TestError("NoMemory: page pressure slots"))?;
            objects
                .try_reserve_exact(PRESSURE_CAPACITY)
                .map_err(|_| TestError("NoMemory: heap pressure slots"))?;
        }
        Ok(Self {
            initial,
            state,
            handles,
            pages,
            objects,
            oom_pages: None,
        })
    }
    /// 合作停止所有真实循环并通知它们已有等待；压力拥有留到原join离栈以后。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.state.stop.store(true, Ordering::Release);
        self.state.drain_running.store(false, Ordering::Release);
        self.state.queue.wake_all();
        self.state.drain_queue.wake_all();
        for slot in &self.state.slots {
            let waker = slot.waker.lock().as_ref().cloned();
            if let Some(waker) = waker {
                waker.notify();
            }
        }
        for _ in 0..TASKS {
            self.state.start_oom.complete();
            self.state.finish_oom.complete();
            self.state.tokens.complete();
        }
        crate::ktest::execution_core::until(
            || {
                self.handles
                    .iter()
                    .all(|handle| handle.try_join().is_some())
            },
            "wait stop did not reach original off-stack completion",
        )?;
        if self.state.queue.test_members() != 0
            || self.state.drain_queue.test_members() != 0
            || [
                &self.state.start_oom,
                &self.state.finish_oom,
                &self.state.tokens,
            ]
            .iter()
            .any(|source| source.test_waiting() != 0)
        {
            return Err(TestError("wait stop retains original stack registrations"));
        }
        Ok(())
    }
    /// 原scope的真实join已结束；先去观察/压力/源，再核本次独立六域和metadata。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        let Self {
            initial,
            state,
            handles,
            mut pages,
            mut objects,
            oom_pages,
        } = self;
        if handles.iter().any(|handle| handle.try_join().is_none()) {
            return Err(TestError("waiting Task still on execution stack"));
        }
        if state.queue.test_members() != 0 || state.drain_queue.test_members() != 0 {
            return Err(TestError("wait source still owns stack nodes"));
        }
        for slot in &state.slots {
            slot.clear();
        }
        let old = state.old_waker.lock().take();
        drop(old);
        for source in [&state.start_oom, &state.finish_oom, &state.tokens] {
            if source.test_waiting() != 0 {
                return Err(TestError("completion retains live wait nodes"));
            }
            while source.try_wait() {}
            assert!(!source.is_ready());
        }
        assert!(state
            .slots
            .iter()
            .all(|slot| slot.task.lock().is_none() && slot.waker.lock().is_none()));
        drop(handles);
        objects.clear();
        pages.clear();
        if let Some(expected) = oom_pages {
            let actual = baseline()?.0;
            if actual.buddy_free_pages + actual.pcp_cached_pages + actual.slab_backing_pages
                != expected
            {
                return Err(TestError(
                    "OOM page accounting did not recover after original join",
                ));
            }
        }
        drop(objects);
        drop(pages);
        drop(state);
        reclaimed(initial)
    }
}
impl Waiting {
    /// 观察handle归本fixture，真实原join责任仍由scope强拥有且不得丢失。
    fn spawn(
        &mut self,
        cpu: usize,
        body: impl FnOnce(Arc<Control>, ChildToken) -> Result<i32, TestError> + Send + 'static,
    ) -> Result<usize, TestError> {
        let state = Arc::clone(&self.state);
        let handle = spawn_result_on(cpu, move |token| body(state, token))?;
        assert!(self.handles.len() < self.handles.capacity());
        let index = self.handles.len();
        self.handles.push(handle);
        Ok(index)
    }
    /// Any任务仍由生产准入选择CPU，控制Arc独立拥有而不借fixture栈。
    fn spawn_any(
        &mut self,
        body: impl FnOnce(Arc<Control>, ChildToken) -> Result<i32, TestError> + Send + 'static,
    ) -> TestResult {
        let state = Arc::clone(&self.state);
        let handle = spawn_result(move |token| body(state, token))?;
        assert!(self.handles.len() < self.handles.capacity());
        self.handles.push(handle);
        Ok(())
    }
}

/// 核对真实Blocked和零ready成员，不拿测试phase代替调度事实。
fn blocked(slot: &Slot, phase: usize) {
    until(
        || {
            slot.phase.load(Ordering::Acquire) == phase
                && slot
                    .task
                    .lock()
                    .as_ref()
                    .is_some_and(|task| sched::test_state(task) == State::Blocked)
        },
        "wait-worker-blocked",
    );
    assert_eq!(sched::test_ready_occurrences(&slot.task()), 0);
}

/// 真实yield与源登记park混合，原Task始终只有一个执行者。
fn mixed_worker(state: Arc<Control>, token: ChildToken, index: usize) -> Result<i32, TestError> {
    let slot = &state.slots[index];
    slot.publish_task();
    for round in 1..=ROUNDS {
        if state.stop.load(Ordering::Acquire) || token.stop_requested() {
            break;
        }
        let active = slot.active.fetch_add(1, Ordering::AcqRel) + 1;
        slot.maximum.fetch_max(active, Ordering::AcqRel);
        assert_eq!(active, 1);
        sched::yield_now();
        slot.switches.fetch_add(1, Ordering::Relaxed);
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: 本入口持原state强拥有；node固定于本Task栈，registration从不遗忘且先撤链再退栈。
        let registration = unsafe { state.queue.register(node.as_mut()) };
        let previous = slot.waker.lock().replace(waiter.waker());
        drop(previous);
        slot.phase.store(round, Ordering::Release);
        if !state.stop.load(Ordering::Acquire) && !token.stop_requested() {
            waiter.park();
        }
        slot.switches.fetch_add(1, Ordering::Relaxed);
        assert_eq!(
            sched::test_state(&slot.task()),
            State::Running(cpu::current().id)
        );
        assert_eq!(sched::test_ready_occurrences(&slot.task()), 0);
        assert_eq!(slot.active.fetch_sub(1, Ordering::AcqRel), 1);
        drop(registration);
        drop(waiter);
    }
    Ok((slot.switches.load(Ordering::Acquire) / 2) as i32)
}

/// 确认已登记一轮等待，可选择yield提交Ready窗口或保留旧Waker制造新代次。
fn window_worker(
    state: Arc<Control>,
    token: ChildToken,
    ready: bool,
    generation: bool,
) -> Result<i32, TestError> {
    let slot = &state.slots[0];
    slot.publish_task();
    if generation {
        let old_id = {
            let waiter = Waiter::new();
            let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
            // SAFETY: 原state保源，node固定；A登记先真实撤链再销毁Waiter，弱通知不保节点访问权。
            let registration = unsafe { state.queue.register(node.as_mut()) };
            let previous = state.old_waker.lock().replace(waiter.waker());
            drop(previous);
            let id = waiter.id();
            drop(registration);
            id
        };
        let waiter = Waiter::new();
        assert_ne!(old_id, waiter.id());
        park_window(&state, &token, slot, &waiter, false);
    } else {
        let waiter = Waiter::new();
        park_window(&state, &token, slot, &waiter, ready);
    }
    slot.phase.store(2, Ordering::Release);
    Ok(if generation { 2 } else { 1 })
}

/// 登记后的停止重检与真实通知共用原Waiter，关门不能丢掉随后park的唤醒。
fn park_window(state: &Control, token: &ChildToken, slot: &Slot, waiter: &Waiter, ready: bool) {
    let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
    // SAFETY: 调用者的Arc保state，node固定且唯一registration覆盖全部park/yield，退出先撤链。
    let registration = unsafe { state.queue.register(node.as_mut()) };
    let previous = slot.waker.lock().replace(waiter.waker());
    drop(previous);
    slot.phase.store(1, Ordering::Release);
    if ready {
        sched::yield_now();
    }
    if !state.stop.load(Ordering::Acquire) && !token.stop_requested() {
        waiter.park();
    }
    assert_eq!(
        sched::test_state(&slot.task()),
        State::Running(cpu::current().id)
    );
    drop(registration);
}

/// 每次真实pop恢复后结束旧轮并再登记，停止只能在撤链后的普通循环边界生效。
fn drain_worker(state: Arc<Control>, token: ChildToken, index: usize) -> Result<i32, TestError> {
    let slot = &state.slots[index];
    slot.publish_task();
    let mut round = 0;
    while state.drain_running.load(Ordering::Acquire) && !token.stop_requested() {
        let waiter = Waiter::new();
        let mut node = core::pin::pin!(WaitNode::new(waiter.waker()));
        // SAFETY: 原Arc保源，node在此Task栈固定；每轮显式撤链后结束Waiter，不移交栈借用。
        let registration = unsafe { state.drain_queue.register(node.as_mut()) };
        round += 1;
        if round == 1 {
            state.drain_initial.fetch_add(1, Ordering::Release);
        } else {
            state.drain_rearms.fetch_add(1, Ordering::Release);
        }
        slot.phase.store(round, Ordering::Release);
        if state.drain_running.load(Ordering::Acquire) && !token.stop_requested() {
            waiter.park();
        }
        drop(registration);
        drop(waiter);
    }
    Ok(round as i32)
}

/// 所有压力槽已准备；页分配真正返回None才结束该order，容量满不能算OOM。
fn exhaust_pages(fixture: &mut Waiting) -> usize {
    let mut held_pages = 0;
    mm::drain_all_pages();
    for order in (0..=mm::MAX_ORDER).rev() {
        loop {
            assert!(
                fixture.pages.len() < fixture.pages.capacity(),
                "execution page pressure storage is full"
            );
            let Some(pages) = mm::alloc_pages(order, false) else {
                break;
            };
            held_pages += pages.len() / mm::PAGE_SIZE;
            fixture.pages.push(pages);
        }
    }
    assert!(0 < held_pages);
    assert!(mm::alloc_pages(0, false).is_none());
    held_pages
}

/// 消费十二SLUB类的真实剩余对象，成功原拥有直接放入catch外具体fixture。
fn exhaust_objects(fixture: &mut Waiting) -> usize {
    let mut nulls = 0;
    for size in CLASSES.iter().copied() {
        let layout = Layout::from_size_align(size, 8).unwrap();
        loop {
            assert!(
                fixture.objects.len() < fixture.objects.capacity(),
                "execution object pressure storage is full"
            );
            // SAFETY: MM已初始化且layout有效非零，成功结果立即交给唯一HeapOwner按原布局归还。
            let pointer = unsafe { alloc(layout) };
            if pointer.is_null() {
                nulls += 1;
                break;
            }
            fixture.objects.push(HeapOwner { pointer, layout });
        }
    }
    nulls
}

/// 保持真实压力时再次请求页和每堆类，意外成功先归还原对象再报告失败。
fn verify_oom() -> usize {
    assert!(mm::alloc_pages(0, false).is_none());
    let mut nulls = 1;
    for size in CLASSES.iter().copied() {
        let layout = Layout::from_size_align(size, 8).unwrap();
        // SAFETY: 有效layout的标准GlobalAlloc入口；意外成功立即拥有，失败无归还权。
        let pointer = unsafe { alloc(layout) };
        let recovered = !pointer.is_null();
        if recovered {
            drop(HeapOwner { pointer, layout });
        }
        assert!(
            !recovered,
            "execution allocator recovered before pressure release"
        );
        nulls += 1;
    }
    nulls
}

/// 真实OOM期间预备Task争SleepMutex，恢复后仍保其原栈直到压力释放门开启。
fn oom_worker(state: Arc<Control>, _: ChildToken) -> Result<i32, TestError> {
    let slot = &state.slots[0];
    slot.publish_task();
    slot.phase.store(1, Ordering::Release);
    state.start_oom.wait();
    if state.stop.load(Ordering::Acquire) {
        return Ok(0);
    }
    slot.phase.store(2, Ordering::Release);
    let mut guard = state.payload.lock();
    *guard += 1;
    state.oom_progress.fetch_add(1, Ordering::Release);
    drop(guard);
    state.finish_oom.wait();
    Ok(1)
}

/// 原Any竞争者在持有guard时真实迁核和park，Release解锁前其他Task不能取得占有。
fn payload_worker(state: Arc<Control>, token: ChildToken, index: usize) -> Result<i32, TestError> {
    sched::test_migrate_to(0);
    let slot = &state.slots[index];
    slot.publish_task();
    assert!(state.payload.try_lock().is_none());
    state.failed_try.fetch_add(1, Ordering::Relaxed);
    slot.phase.store(1, Ordering::Release);
    let mut updates = 0;
    for round in 0..ROUNDS {
        if state.stop.load(Ordering::Acquire) || token.stop_requested() {
            break;
        }
        let mut guard = state.payload.lock();
        let before = *guard;
        if index == 0 && round == 0 {
            if 1 < cpu::count() {
                sched::test_migrate_to(1);
                assert_eq!(cpu::current().id, 1);
                state.migrations.fetch_add(1, Ordering::Relaxed);
                assert_eq!(*guard, before);
                sched::test_migrate_to(0);
                assert_eq!(cpu::current().id, 0);
                state.migrations.fetch_add(1, Ordering::Relaxed);
            }
            let waiter = Waiter::new();
            let previous = slot.waker.lock().replace(waiter.waker());
            drop(previous);
            slot.phase.store(2, Ordering::Release);
            if !state.stop.load(Ordering::Acquire) && !token.stop_requested() {
                waiter.park();
            }
            assert_eq!(*guard, before);
            state.parked.fetch_add(1, Ordering::Release);
        }
        assert_eq!(*guard, before);
        *guard = before + 1;
        updates += 1;
        drop(guard);
        sched::yield_now();
    }
    Ok(updates)
}

/// 真实Completion等待只有消费到token才返回；quiesce用具体token唤醒原Task。
fn token_worker(state: Arc<Control>, _: ChildToken, index: usize) -> Result<i32, TestError> {
    let slot = &state.slots[index];
    slot.publish_task();
    slot.phase.store(1, Ordering::Release);
    state.tokens.wait();
    state.consumed.fetch_add(1, Ordering::Release);
    slot.phase.store(2, Ordering::Release);
    Ok(1)
}

/// 三页域合计只检查活栈差；每case最后仍执行六域和元数据独立比较。
fn page_total() -> usize {
    let stats = mm::stats();
    stats.buddy_free_pages + stats.pcp_cached_pages + stats.slab_backing_pages
}

#[::ktest::suite(fixture = Waiting, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "wait"), timeout_ms = 20000)]
pub(crate) mod queue {
    use super::*;
    /// 四个生产Task共32次真实恢复，active最大一且ready成员不会重复。
    #[test]
    fn exclusive_ready_membership(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        for index in 0..TASKS {
            fixture.spawn_any(move |state, token| mixed_worker(state, token, index))?;
        }
        let state = &fixture.state;
        let mut ready_max = 0;
        for round in 1..=ROUNDS {
            for slot in &state.slots {
                blocked(slot, round);
                assert_eq!(slot.active.load(Ordering::Acquire), 1);
                let waker = slot
                    .waker
                    .lock()
                    .as_ref()
                    .expect("mixed worker waker not published")
                    .clone();
                assert!(waker.notify());
                assert!(!waker.notify());
                let ready = sched::test_ready_occurrences(&slot.task());
                assert!(ready <= 1);
                ready_max = ready_max.max(ready);
            }
        }
        for handle in &fixture.handles {
            assert_eq!(join(handle)?, ROUNDS as i32);
        }
        assert_eq!(
            state
                .slots
                .iter()
                .map(|slot| slot.maximum.load(Ordering::Acquire))
                .max(),
            Some(1)
        );
        assert_eq!(
            state
                .slots
                .iter()
                .map(|slot| slot.switches.load(Ordering::Acquire))
                .sum::<usize>(),
            TASKS * ROUNDS * 2
        );
        if cpu::count() == 1 {
            assert_eq!(ready_max, 1);
        } else {
            assert!(ready_max <= 1);
        }
        for slot in &state.slots {
            assert_eq!(slot.active.load(Ordering::Acquire), 0);
        }
        assert_eq!(state.queue.wake_all(), 0);
        Ok(())
    }

    /// 同一原WaitId在源登记前或后通知，park都立即返回，重复通知拒绝。
    #[test]
    #[case("before_source_registration", false)]
    #[case("after_source_registration", true)]
    fn early_notification(_: &mut TestContext<'_>, fixture: &mut Waiting, after: bool) {
        let waiter = Waiter::new();
        let waker = waiter.waker();
        let mut node = core::pin::pin!(WaitNode::new(waker.clone()));
        let accepted;
        if !after {
            accepted = waker.notify();
        } else {
            accepted = false;
        }
        // SAFETY: fixture保源且局部node固定，唯一registration显式撤链后才结束Waiter和栈帧。
        let registration = unsafe { fixture.state.queue.register(node.as_mut()) };
        let accepted = if after {
            fixture.state.queue.wake_one()
        } else {
            accepted
        };
        assert!(accepted);
        assert!(!waker.notify());
        waiter.park();
        assert_eq!(
            sched::test_state(&task::current_task()),
            State::Running(cpu::current().id)
        );
        drop(registration);
        assert_eq!(fixture.state.queue.test_members(), 0);
    }

    /// 当前真实Task已提交SwitchingOut尚未保存ctx时通知，不提前加入ready且恢复后拒重复。
    #[test]
    fn switching_out_notification(_: &mut TestContext<'_>, _: &mut Waiting) {
        let waiter = Waiter::new();
        waiter.test_park_notify();
        assert!(!waiter.waker().notify());
    }

    /// 固定同核协调者保住Ready窗口；普通suite仍为Any，不让远核提前消费被观察者。
    #[test]
    #[case("blocked", false)]
    #[case("ready", true)]
    fn queued_notification(
        _: &mut TestContext<'_>,
        fixture: &mut Waiting,
        ready: bool,
    ) -> TestResult {
        let coordinator = fixture.spawn(0, move |state, _| {
            let source = Arc::clone(&state);
            let worker =
                spawn_result_on(0, move |token| window_worker(source, token, ready, false))?;
            let slot = &state.slots[0];
            if ready {
                until(
                    || {
                        slot.phase.load(Ordering::Acquire) == 1
                            && slot
                                .task
                                .lock()
                                .as_ref()
                                .is_some_and(|task| sched::test_state(task) == State::Ready)
                    },
                    "wait-worker-ready",
                );
                assert_eq!(sched::test_ready_occurrences(&slot.task()), 1);
            } else {
                blocked(slot, 1);
            }
            assert!(slot.notify());
            assert!(!slot.notify());
            assert_eq!(sched::test_state(&slot.task()), State::Ready);
            assert_eq!(sched::test_ready_occurrences(&slot.task()), 1);
            assert_eq!(join(&worker)?, 1);
            assert_eq!(slot.phase.load(Ordering::Acquire), 2);
            Ok(0)
        })?;
        assert_eq!(join(&fixture.handles[coordinator])?, 0);
        Ok(())
    }

    /// 真CPU1通知真Blocked协调Task，不拿CPU数或phase伪造远端接受。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_notification(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        let slot = &fixture.state.slots[0];
        slot.publish_task();
        let waiter = Waiter::new();
        let previous = slot.waker.lock().replace(waiter.waker());
        drop(previous);
        slot.phase.store(1, Ordering::Release);
        let index = fixture.spawn(1, |state, token| {
            let _wake = WakeOnReturn(Arc::clone(&state));
            until(
                || {
                    state.stop.load(Ordering::Acquire)
                        || token.stop_requested()
                        || sched::test_state(&state.slots[0].task()) == State::Blocked
                },
                "remote parent blocked",
            );
            if state.stop.load(Ordering::Acquire) || token.stop_requested() {
                return Ok(0);
            }
            blocked(&state.slots[0], 1);
            assert_eq!(cpu::current().id, 1);
            Ok(state.slots[0].notify() as i32)
        })?;
        waiter.park();
        drop(waiter);
        assert_eq!(join(&fixture.handles[index])?, 1);
        Ok(())
    }

    /// 两个CPU1 Task每次真实pop后重登记，本次wake_all最多消费原入口两成员预算。
    #[test]
    #[ktest(min_cpus = 2)]
    fn finite_wake_all_rearming(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        for index in 0..2 {
            fixture.spawn(1, move |state, token| drain_worker(state, token, index))?;
            blocked(&fixture.state.slots[index], 1);
        }
        let state = &fixture.state;
        let initial = state.drain_initial.load(Ordering::Acquire);
        assert_eq!(initial, 2);
        let mut pops = 0;
        let accepted = state.drain_queue.test_wake_all_after_notify(|| {
            pops += 1;
            if pops <= initial {
                blocked(&state.slots[pops - 1], 2);
            } else {
                state.drain_running.store(false, Ordering::Release);
            }
        });
        assert_eq!(
            accepted, initial,
            "wake_all consumed newly registered waits"
        );
        assert_eq!(pops, initial, "wake_all exceeded initial member budget");
        assert_eq!(state.drain_rearms.load(Ordering::Acquire), initial);
        state.drain_running.store(false, Ordering::Release);
        assert_eq!(state.drain_queue.wake_all(), initial);
        for handle in &fixture.handles {
            assert_eq!(join(handle)?, 2);
        }
        assert_eq!(state.drain_queue.wake_all(), 0);
        Ok(())
    }

    /// A真实结束后B的新WaitId仍Blocked，保留的旧Waker拒绝且新Waker正常恢复。
    #[test]
    fn stale_waker_new_generation(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        let index = fixture.spawn(0, |state, token| window_worker(state, token, false, true))?;
        let slot = &fixture.state.slots[0];
        blocked(slot, 1);
        let old = fixture.state.old_waker.lock().as_ref().unwrap().clone();
        assert!(!old.notify());
        assert_eq!(slot.phase.load(Ordering::Acquire), 1);
        assert_eq!(sched::test_state(&slot.task()), State::Blocked);
        assert_eq!(sched::test_ready_occurrences(&slot.task()), 0);
        assert!(slot.notify());
        assert_eq!(join(&fixture.handles[index])?, 2);
        assert_eq!(slot.phase.load(Ordering::Acquire), 2);
        drop(old);
        assert_eq!(fixture.state.queue.wake_all(), 0);
        Ok(())
    }

    /// 页和十二堆类真正耗尽期间，原Task仍可登记阻塞、解锁恢复、再次阻塞并最终还原页账。
    #[test]
    fn registration_under_oom(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        mm::shrink_all();
        mm::drain_all_pages();
        fixture.oom_pages = Some(page_total());
        let state = Arc::clone(&fixture.state);
        let guard = state.payload.lock();
        let index = fixture.spawn(0, oom_worker)?;
        blocked(&state.slots[0], 1);
        mm::shrink_all();
        let pages = exhaust_pages(fixture);
        let class_nulls = exhaust_objects(fixture);
        assert_eq!(class_nulls, CLASSES.len());
        let mut nulls = 1 + class_nulls;
        state.start_oom.complete();
        blocked(&state.slots[0], 2);
        assert_eq!(sched::test_state(&state.slots[0].task()), State::Blocked);
        drop(guard);
        until(
            || {
                state.oom_progress.load(Ordering::Acquire) == 1
                    && sched::test_state(&state.slots[0].task()) == State::Blocked
            },
            "OOM-worker-progress",
        );
        nulls += verify_oom();
        assert_eq!(*state.payload.lock(), 1);
        assert!(0 < pages);
        assert_eq!(nulls, 2 * (CLASSES.len() + 1));
        fixture.objects.clear();
        fixture.pages.clear();
        state.finish_oom.complete();
        assert_eq!(
            join(&fixture.handles[index])?,
            state.oom_progress.load(Ordering::Acquire) as i32
        );
        state.slots[0].clear();
        Ok(())
    }
}

#[::ktest::suite(fixture = Waiting, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "sync"), timeout_ms = 20000)]
pub(crate) mod mutex {
    use super::*;
    /// 四真实竞争者在Acquire/Release下更新非原子payload，持guard park也不移交占有。
    #[test]
    fn ownership_visibility(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        let state = Arc::clone(&fixture.state);
        let guard = state.payload.lock();
        for _ in 0..TASKS {
            assert!(state.payload.try_lock().is_none());
            state.failed_try.fetch_add(1, Ordering::Relaxed);
        }
        for index in 0..TASKS {
            fixture.spawn_any(move |state, token| payload_worker(state, token, index))?;
        }
        for slot in &state.slots {
            blocked(slot, 1);
        }
        drop(guard);
        blocked(&state.slots[0], 2);
        assert!(state.payload.try_lock().is_none());
        state.failed_try.fetch_add(1, Ordering::Relaxed);
        assert!(state.slots[0].notify());
        let mut updates = 0;
        for handle in &fixture.handles {
            updates += join(handle)? as usize;
        }
        let visibility = *state.payload.lock();
        assert_eq!(visibility, TASKS * ROUNDS);
        assert_eq!(updates, visibility);
        assert_eq!(state.failed_try.load(Ordering::Acquire), 2 * TASKS + 1);
        assert_eq!(state.parked.load(Ordering::Acquire), 1);
        assert_eq!(
            state.migrations.load(Ordering::Acquire),
            if cpu::count() == 1 { 0 } else { 2 }
        );
        Ok(())
    }

    /// 非递归try_lock失败不释放原guard，再次成功只能晚于原Release析构。
    #[test]
    fn recursive_try_lock_preserves_owner(_: &mut TestContext<'_>, fixture: &mut Waiting) {
        let mut guard = fixture.state.payload.lock();
        *guard = 37;
        assert!(fixture.state.payload.try_lock().is_none());
        assert_eq!(*guard, 37);
        drop(guard);
        assert_eq!(
            *fixture
                .state
                .payload
                .try_lock()
                .expect("released mutex remains occupied"),
            37
        );
    }

    /// 原Task携带不可Send guard迁核再返回，payload借用和原占有跨两次真实切换保持。
    #[test]
    #[ktest(min_cpus = 2)]
    fn guard_migration(_: &mut TestContext<'_>, fixture: &mut Waiting) {
        sched::test_migrate_to(0);
        let id = task::current_task_id();
        let mut guard = fixture.state.payload.lock();
        *guard = 0x7307;
        for target in [1, 0].iter().copied() {
            sched::test_migrate_to(target);
            assert_eq!(cpu::current().id, target);
            assert_eq!(task::current_task_id(), id);
            assert_eq!(*guard, 0x7307);
            assert!(fixture.state.payload.try_lock().is_none());
        }
    }

    /// 四已Blocked消费者各真实消费一个token，无余token或假完成。
    #[test]
    fn completion_tokens(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        sched::test_migrate_to(0);
        for index in 0..TASKS {
            fixture.spawn(0, move |state, token| token_worker(state, token, index))?;
        }
        for slot in &fixture.state.slots {
            blocked(slot, 1);
        }
        for _ in 0..TASKS {
            fixture.state.tokens.complete();
        }
        let mut consumers = 0;
        for handle in &fixture.handles {
            consumers += join(handle)? as usize;
        }
        assert_eq!(consumers, TASKS);
        assert_eq!(fixture.state.consumed.load(Ordering::Acquire), consumers);
        assert!(!fixture.state.tokens.try_wait());
        Ok(())
    }

    /// 同核协调者窃取Ready等待者的token，原等待者必须重新Blocked而不能误完成。
    #[test]
    fn stolen_completion_token(_: &mut TestContext<'_>, fixture: &mut Waiting) -> TestResult {
        let coordinator = fixture.spawn(0, |state, _| {
            let source = Arc::clone(&state);
            let worker = spawn_result_on(0, move |token| token_worker(source, token, 0))?;
            blocked(&state.slots[0], 1);
            state.tokens.complete();
            assert_eq!(sched::test_state(&state.slots[0].task()), State::Ready);
            assert!(state.tokens.try_wait());
            blocked(&state.slots[0], 1);
            assert_eq!(state.consumed.load(Ordering::Acquire), 0);
            state.tokens.complete();
            assert_eq!(join(&worker)?, 1);
            assert_eq!(state.consumed.load(Ordering::Acquire), 1);
            assert!(!state.tokens.try_wait());
            Ok(0)
        })?;
        assert_eq!(join(&fixture.handles[coordinator])?, 0);
        Ok(())
    }
}
