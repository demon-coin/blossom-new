//! 真实固定 CPU Task 的跨核唯一权、PCP 直接 drain 与 SLUB shared 交接。

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

const MAX: usize = crate::boot::MAX_CPUS;
pub(super) const WORK: usize = 32;

pub(crate) struct State {
    pub(crate) stop: AtomicBool,
    pub(crate) ready: AtomicUsize,
    pub(crate) phase: AtomicUsize,
    pub(crate) turn: AtomicUsize,
    drain_epoch: AtomicUsize,
    pub(crate) done: [AtomicUsize; MAX],
    pub(crate) pages: Vec<SpinLock<Option<OwnedPages>>>,
    identities: Vec<AtomicUsize>,
    pub(crate) objects: [SpinLock<Option<SlabObject>>; 96],
    pub(crate) command: AtomicUsize,
    pub(crate) acknowledgement: AtomicUsize,
    drops: AtomicUsize,
    drop_cpu: AtomicUsize,
    payload: SpinLock<Option<Arc<Payload>>>,
}
impl State {
    /// 按实际核数在堆上逐槽准备 owning 表，避免把最大 CPU 页槽数组放到任务栈。
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
    /// 普通 Task 合作等待；超时返回真实错误并由 fixture 停止/框架 join。
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
    /// 已发表完成的参与者保住原 Task/栈到 quiesce，停点账务不与离栈析构竞争。
    pub(crate) fn park(&self, token: &crate::ktest::ChildToken) -> TestResult {
        while !self.stop.load(Ordering::Acquire) && !token.stop_requested() {
            crate::sched::yield_now();
        }
        Ok(())
    }
    /// 原子页身份采样只在发布停点，不复制释放权。
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
    /// 申请完即发布 owning 槽，然后才核验身份和内容。
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
    /// 借用对象身份；消费者必须在发布停点保证 owning 槽不被远端消费。
    pub(crate) fn pointer(&self, index: usize) -> core::ptr::NonNull<u8> {
        self.objects[index].lock().as_ref().unwrap().0
    }
    /// 检查先于 take，析构在控制槽锁外执行真实异核归还。
    pub(crate) fn release_object(&self, index: usize) {
        self.objects[index].lock().as_ref().unwrap().check();
        let object = self.objects[index].lock().take();
        drop(object);
    }
}

pub(crate) struct SmpFixture {
    baseline: MemoryStats,
    pub(crate) state: Arc<State>,
}
impl Fixture for SmpFixture {
    /// 先记录真实常驻六域，再 fallible 准备具体跨核控制存储。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled();
        let state =
            Arc::try_new(State::new()?).map_err(|_| TestError("NoMemory: SMP memory controls"))?;
        Ok(Self { baseline, state })
    }
    /// 关闭具体合作源；框架随后一次 stop_children 和真实 join 全部原 Task。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.state.stop.store(true, Ordering::Release);
        Ok(())
    }
    /// 所有原 Task 已离栈，消费仍存 owning 槽及控制容量后核六域。
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
    /// 用真实固定核 Task 发表拥有的具体控制对象，不传栈借用。
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
    /// 控制者允许 scheduler 运行同核参与者；失收敛交框架 case-abort，绝不收活页。
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
    /// 发布停点的全部参与者已完成对应原动作。
    fn all_done(&self, phase: usize) -> TestResult {
        self.until(|| {
            self.state.done[..cpu::count()]
                .iter()
                .all(|d| d.load(Ordering::Acquire) == phase)
        })
    }
}

#[ktest::suite(fixture = SmpFixture, tags("mm", "smp", "pcp"), timeout_ms = 30000)]
mod pages {
    use super::*;
    /// 每核真实页保 live 去重，邻核最后 Drop 只增加实际释放 CPU 的 PCP；单核保原 local 路径。
    #[test]
    fn final_drop_cpu(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        for id in 0..cpu::count() {
            f.spawn(id, |state, token| {
                let id = cpu::current().id;
                state.wait(&token, || state.phase.load(Ordering::Acquire) == 1)?;
                for index in 0..8 {
                    let mut pages = mm::alloc_pages(0, false).unwrap();
                    state.identities[id * WORK + index].store(pages.phys_addr(), Ordering::Relaxed);
                    // SAFETY: 原页独占且未发布，只写首 usize；release/acquire 停点后邻核才读取。
                    unsafe { pages.as_mut_ptr().cast::<usize>().write(id * 8 + index) };
                    *state.pages[id * WORK + index].lock() = Some(pages);
                }
                state.done[id].store(1, Ordering::Release);
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
                    // SAFETY: owning 槽保原页到内容检查结束，source 已初始化首 word，全部源访问结束。
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
    /// 每个在线 CPU 依次到真实 high 阈值，恰返回一个 batch，停止后排空。
    #[test]
    fn every_cpu_high(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        for id in 0..cpu::count() {
            f.spawn(id, |state, token| {
                let id = cpu::current().id;
                let (batch, high) = pcp::parameters();
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
    /// 远端留真实 batch 缓存并停稳，控制者直接 drain 不依赖 IPI。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_cache_drain(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        for id in 0..cpu::count() {
            f.spawn(id, |state, token| {
                let id = cpu::current().id;
                state.wait(&token, || state.phase.load(Ordering::Acquire) == 1)?;
                let (batch, _) = pcp::parameters();
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
        let (batch, _) = pcp::parameters();
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
    /// 十六轮真实 AP 分配/释放与直接 drain 同时运行，停点仍核 live 唯一及完整账务。
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
                            && phase + 1 <= state.drain_epoch.load(Ordering::Acquire)
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
    /// 每在线 CPU 的原 Task 分别复用 IRQ 开/关两种锁恢复断言，不借用控制者栈。
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

#[ktest::suite(fixture = SmpFixture, tags("mm", "smp", "slub"), min_cpus = 2, timeout_ms = 30000)]
mod objects {
    use super::*;
    /// 真正异核 free 只写 shared，owner local 命中与下一对 remote free 并行。
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

struct Payload {
    words: [usize; 12],
    state: alloc::sync::Weak<State>,
}
impl Payload {
    /// 只读取原 Arc 保活期间的消费者数据。
    fn check(&self) {
        for (i, &word) in self.words.iter().enumerate() {
            assert_eq!(word, i * 37 + 11);
        }
    }
}
impl Drop for Payload {
    /// 析构记录实际 CPU，不分配也不在 Drop 内执行可能 panic 的内容断言。
    fn drop(&mut self) {
        if let Some(state) = self.state.upgrade() {
            state.drop_cpu.store(cpu::current().id, Ordering::Relaxed);
            state.drops.fetch_add(1, Ordering::Release);
        }
    }
}

#[ktest::suite(fixture = SmpFixture, tags("mm", "heap", "arc"), timeout_ms = 30000)]
mod arc_ownership {
    use super::*;
    /// 本核最后 Arc Drop 的次数和真实 CPU 独立于异核参数。
    #[test]
    fn local_final_drop(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        final_drop(f, 0)
    }
    /// 最后一份 Arc 交给真实 CPU1 Task，不能用同步 callback 冒充普通析构流。
    #[test]
    #[ktest(min_cpus = 2)]
    fn remote_final_drop(_: &mut TestContext<'_>, f: &mut SmpFixture) -> TestResult {
        final_drop(f, 1)
    }
    /// 具体 owning 槽从创建起保留 payload，最后权通过固定核原 Task 消费。
    fn final_drop(f: &mut SmpFixture, target: usize) -> TestResult {
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
        f.until(|| f.state.done[target].load(Ordering::Acquire) == 1)?;
        assert_eq!(f.state.drops.load(Ordering::Acquire), 1);
        assert_eq!(f.state.drop_cpu.load(Ordering::Relaxed), target);
        assert!(f.state.payload.lock().is_none());
        Ok(())
    }
}
