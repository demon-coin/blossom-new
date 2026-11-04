//! 页与原布局堆对象由具体 fixture 保活，真实迁核后在目的核归还。

use crate::ktest::execution_core::{baseline, reclaimed, Baseline};
use crate::ktest::execution_services::SuiteBaseline;
use crate::ktest::{spawn_result, ChildHandle};
use crate::mm::SLUB_SIZES as CLASSES;
use crate::sync::SleepMutex;
use crate::{arch, cpu, mm, sched, task};
use ::ktest::{Fixture, TestContext, TestError, TestResult};
use alloc::alloc::{alloc, dealloc};
use alloc::sync::Arc;
use core::alloc::Layout;
use core::hint::black_box;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicBool, Ordering};

const PRESSURE_PAGES: usize = 64;
const PRESSURE_OBJECTS: usize = 96;

#[derive(Default)]
struct Counts {
    task: usize,
    visited: cpu::CpuSet,
    hops: usize,
    local_rounds: usize,
    page_allocs: usize,
    page_frees: usize,
    heap_allocs: usize,
    heap_frees: usize,
    flushes: usize,
    pressure_pages: usize,
    pressure_objects: usize,
    content_checks: usize,
    cpu_checks: usize,
    stack_checks: usize,
}

struct StackWitness {
    address: usize,
    physical: (usize, usize),
}

struct HeapObject {
    pointer: NonNull<u8>,
    layout: Layout,
    pattern: u8,
}
// SAFETY: HeapObject独占原全局堆分配，不导出可逃逸的指针借用且不可复制；
// allocator允许原布局在其他CPU/Task归还，内容访问只由持有Payload唯一guard的Task进行。
unsafe impl Send for HeapObject {}

impl HeapObject {
    /// 从标准全局堆取得原布局对象并写满内容，容量不足直接使本次验收失败。
    fn new(size: usize, pattern: u8) -> Self {
        let layout = Layout::from_size_align(size, 8).expect("X24 heap layout");
        // SAFETY: 固定类别均非零且 Layout 合法，唯一 MM 已初始化；普通任务不持锁或 IRQ guard。
        let pointer = NonNull::new(unsafe { alloc(layout) }).expect("X24 heap allocation failed");
        // SAFETY: alloc 返回满足 layout 的 live 独占对象，全部 size 字节均属于该分配；
        // 对象尚未发表给其他执行者，写入结束后才允许读取或归还。
        unsafe { pointer.as_ptr().write_bytes(pattern, size) };
        Self {
            pointer,
            layout,
            pattern,
        }
    }

    /// 在原拥有者仍存活时逐字节验证内容，借用在对象归还之前结束。
    fn check(&self) {
        // SAFETY: new 已初始化原布局的完整范围，当前任务独占对象且没有并发写者；
        // 本次只读切片不逃逸，Drop 尚未执行。
        let bytes = unsafe {
            core::slice::from_raw_parts(black_box(self.pointer.as_ptr()), self.layout.size())
        };
        assert!(
            bytes.iter().all(|byte| *byte == self.pattern),
            "X24 heap content changed"
        );
    }
}

impl Drop for HeapObject {
    /// 按保存的原布局交出唯一归还权，全部内容借用已在普通任务中结束。
    fn drop(&mut self) {
        // SAFETY: pointer 是 new 成功 alloc 返回的原首址，layout 从未修改；
        // 本对象不可复制，仍 live 且全部访问已结束，dealloc 只在唯一 Drop 中执行一次。
        unsafe { dealloc(self.pointer.as_ptr(), self.layout) };
    }
}

/// 取得唯一连续块并初始化完整字节范围，不将 MM 内部短锁或 CPU 借用带到调用者。
fn new_page(order: usize, pattern: u8, counts: &mut Counts) -> mm::OwnedPages {
    let mut pages = mm::alloc_pages(order, false).expect("X24 page allocation failed");
    assert_eq!(pages.order(), order);
    assert_eq!(pages.phys_addr() % pages.len(), 0);
    let len = pages.len();
    // SAFETY: pages 是生产入口返回的独占 live 连续块，直映永久可写，len 是完整块长度；
    // 页入口内部 guard 已结束，写入不与其他执行者共享，也未构造未初始化字节的引用。
    unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(pattern, len) };
    counts.page_allocs += 1;
    pages
}

/// 在一轮 MM 操作前后核对硬件 CPU 基址、同一任务身份、栈范围与局部对象。
fn check_execution(target: usize, local: &usize, witness: &StackWitness, counts: &mut Counts) {
    assert_eq!(cpu::current().id, target);
    assert_eq!(arch::cpu_local(), cpu::get(target) as *const _);
    assert_eq!(cpu::current() as *const _, arch::cpu_local());
    assert_eq!(task::current_task().id().as_usize(), counts.task);
    assert_eq!(
        task::current_task_id().map(|id| id.as_usize()),
        Some(counts.task)
    );
    counts.visited.insert(target);
    counts.cpu_checks += 1;

    assert_eq!(black_box(local) as *const _ as usize, witness.address);
    assert_eq!(*black_box(local), 0x7324);
    assert_eq!(task::test_current_stack_range(), witness.physical);
    let physical = arch::virt_to_phys(witness.address);
    assert!(witness.physical.0 <= physical);
    assert!(physical + core::mem::size_of::<usize>() <= witness.physical.0 + witness.physical.1);
    counts.stack_checks += 1;
}

/// 在已预备的记录槽中持有单页、高阶页和全部十二类标准堆对象。
fn allocate_round(
    pages: &mut [Option<mm::OwnedPages>; PRESSURE_PAGES],
    objects: &mut [Option<HeapObject>; PRESSURE_OBJECTS],
    pattern: u8,
    counts: &mut Counts,
) {
    assert!(pages.iter().all(Option::is_none));
    assert!(objects.iter().all(Option::is_none));
    pages[0] = Some(new_page(0, pattern, counts));
    pages[1] = Some(new_page(3, pattern, counts));
    for (slot, size) in objects.iter_mut().zip(CLASSES.iter().copied()) {
        *slot = Some(HeapObject::new(size, pattern));
        counts.heap_allocs += 1;
    }
}

/// 核验全部拥有对象的内容后逐个归还，最后一次访问与 Drop 均在锁外。
fn check_and_release(
    pages: &mut [Option<mm::OwnedPages>; PRESSURE_PAGES],
    objects: &mut [Option<HeapObject>; PRESSURE_OBJECTS],
    pattern: u8,
    counts: &mut Counts,
) {
    for slot in pages.iter_mut() {
        let Some(mut pages) = slot.take() else {
            continue;
        };
        let len = pages.len();
        // SAFETY: new_page 已写满整个 len 字节，当前任务仍持唯一连续块拥有；
        // 没有其他写者，只读切片在本块 Drop 前结束且不跨任务切换。
        let bytes =
            unsafe { core::slice::from_raw_parts(black_box(pages.as_mut_ptr().cast::<u8>()), len) };
        assert!(
            bytes.iter().all(|byte| *byte == pattern),
            "X24 page content changed"
        );
        counts.content_checks += 1;
        drop(pages);
        counts.page_frees += 1;
    }
    for slot in objects.iter_mut() {
        let Some(object) = slot.take() else {
            continue;
        };
        object.check();
        counts.content_checks += 1;
        drop(object);
        counts.heap_frees += 1;
    }
}

/// 普通可服务 IPI 的任务同步摘离各 CPU 缓存，长归并仍由调用任务完成。
fn flush(counts: &mut Counts) {
    mm::flush_all();
    counts.flushes += 1;
}

/// 同时持有固定六十四页与九十六个对象，排空 live 缓存后核验并完整释放。
fn pressure(
    pages: &mut [Option<mm::OwnedPages>; PRESSURE_PAGES],
    objects: &mut [Option<HeapObject>; PRESSURE_OBJECTS],
    counts: &mut Counts,
) {
    assert!(pages.iter().all(Option::is_none));
    assert!(objects.iter().all(Option::is_none));
    for slot in pages.iter_mut() {
        let owned = new_page(0, 0xd4, counts);
        counts.pressure_pages += owned.len() / mm::PAGE_SIZE;
        *slot = Some(owned);
    }
    for (index, slot) in objects.iter_mut().enumerate() {
        *slot = Some(HeapObject::new(CLASSES[index % CLASSES.len()], 0xd4));
        counts.heap_allocs += 1;
        counts.pressure_objects += 1;
    }
    flush(counts);
    check_and_release(pages, objects, 0xd4, counts);
    flush(counts);
}

struct Payload {
    pages: [Option<mm::OwnedPages>; PRESSURE_PAGES],
    objects: [Option<HeapObject>; PRESSURE_OBJECTS],
    counts: Counts,
}
struct Control {
    payload: SleepMutex<Payload>,
    stop: AtomicBool,
}
struct Memory {
    initial: Baseline,
    state: Arc<Control>,
    handle: Option<ChildHandle>,
}
impl Fixture<SuiteBaseline> for Memory {
    /// 记录槽先于主体捕获存在，任何页或原布局对象的归还权不会随主体panic消失。
    fn set_up(_: &mut TestContext<'_>, _: &SuiteBaseline) -> Result<Self, TestError> {
        let initial = baseline()?;
        let state = Arc::try_new(Control {
            payload: SleepMutex::new(Payload {
                pages: core::array::from_fn(|_| None),
                objects: core::array::from_fn(|_| None),
                counts: Counts::default(),
            }),
            stop: AtomicBool::new(false),
        })
        .map_err(|_| TestError("NoMemory: memory consumer controls"))?;
        Ok(Self {
            initial,
            state,
            handle: None,
        })
    }
    /// 迁核入口只有有限轮次；保住所有原页堆拥有直到其真实离栈，不超时收栈。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.state.stop.store(true, Ordering::Release);
        crate::ktest::execution_core::until(
            || {
                self.handle
                    .as_ref()
                    .map_or(true, |handle| handle.try_join().is_some())
            },
            "memory consumer did not reach original off-stack completion",
        )
    }
    /// 原scope已经join；最后控制owner释放页和原布局对象后逐字段比较六域。
    fn tear_down(self, _: &mut TestContext<'_>) -> TestResult {
        let Self {
            initial,
            state,
            handle,
        } = self;
        drop(handle);
        drop(state);
        reclaimed(initial)
    }
}

/// 同一真实Task建立栈见证，原页和堆对象由catch外具体fixture独占持有。
fn consume(fixture: &mut Payload, migrate: bool, stop: &AtomicBool) {
    let local = 0x7324usize;
    let witness = StackWitness {
        address: black_box(&local) as *const _ as usize,
        physical: task::test_current_stack_range(),
    };
    let Payload {
        pages,
        objects,
        counts,
    } = fixture;
    counts.task = task::current_task().id().as_usize();
    sched::test_migrate_to(0);
    check_execution(0, &local, &witness, counts);
    for round in 0..2 {
        if stop.load(Ordering::Acquire) {
            return;
        }
        let pattern = 0x24 + round as u8;
        allocate_round(pages, objects, pattern, counts);
        check_and_release(pages, objects, pattern, counts);
        flush(counts);
        check_execution(0, &local, &witness, counts);
        counts.local_rounds += 1;
    }
    if migrate {
        for hop in 1..=2 * cpu::count() {
            if stop.load(Ordering::Acquire) {
                return;
            }
            let pattern = 0x40u8.wrapping_add(hop as u8);
            let origin = cpu::current().id;
            let target = hop % cpu::count();
            assert_ne!(origin, target);
            allocate_round(pages, objects, pattern, counts);
            sched::test_migrate_to(target);
            check_execution(target, &local, &witness, counts);
            counts.hops += 1;
            check_and_release(pages, objects, pattern, counts);
            flush(counts);
        }
    }
    assert!(pages.iter().all(Option::is_none));
    assert!(objects.iter().all(Option::is_none));
    assert_eq!(counts.task, task::current_task().id().as_usize());
    assert_eq!(
        counts.visited,
        if migrate {
            cpu::actual_set()
        } else {
            cpu::CpuSet::single(0)
        }
    );
    assert_eq!(counts.hops, if migrate { 2 * cpu::count() } else { 0 });
    assert_eq!(counts.local_rounds, 2);
    assert_eq!(counts.page_allocs, counts.page_frees);
    assert_eq!(counts.heap_allocs, counts.heap_frees);
    assert!(2 * (counts.hops + 2) <= counts.page_allocs);
    assert!(CLASSES.len() * (counts.hops + 2) <= counts.heap_allocs);
    assert!(counts.hops + 2 <= counts.flushes);
    assert!(counts.page_allocs + counts.heap_allocs <= counts.content_checks);
    assert_eq!(counts.cpu_checks, counts.stack_checks);
    assert!(counts.hops + 3 <= counts.cpu_checks);
}

#[::ktest::suite(fixture = Memory, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "mm"), timeout_ms = 20000)]
pub(crate) mod memory {
    use super::*;
    /// 两轮本核真页、order3和十二堆类内容与归还，不要求多核或前例准备。
    #[test]
    fn local_owners(_: &mut TestContext<'_>, fixture: &mut Memory) {
        consume(
            &mut fixture.state.payload.lock(),
            false,
            &fixture.state.stop,
        );
    }

    /// 同Task携带已拥有原页和原布局对象往返全部核两轮，目的核释放并同步排空。
    #[test]
    #[ktest(min_cpus = 2)]
    fn migrating_owners(_: &mut TestContext<'_>, fixture: &mut Memory) -> TestResult {
        let state = Arc::clone(&fixture.state);
        fixture.handle = Some(spawn_result(move |_| {
            consume(&mut state.payload.lock(), true, &state.stop);
            Ok(24)
        })?);
        let handle = fixture.handle.as_ref().unwrap();
        crate::ktest::execution_core::until(
            || handle.try_join().is_some(),
            "memory performer did not reach original off-stack completion",
        )?;
        assert_eq!(handle.join()?, 24);
        let payload = fixture.state.payload.lock();
        assert_eq!(payload.counts.task, handle.task_id().as_usize());
        Ok(())
    }

    /// 六十四页与九十六真堆对象同时存活跨flush，逐字节检查后完整归还。
    #[test]
    fn simultaneous_pressure(_: &mut TestContext<'_>, fixture: &mut Memory) {
        let mut payload = fixture.state.payload.lock();
        let Payload {
            pages,
            objects,
            counts,
        } = &mut *payload;
        pressure(pages, objects, counts);
        assert!(PRESSURE_PAGES <= counts.pressure_pages);
        assert!(PRESSURE_OBJECTS <= counts.pressure_objects);
        assert_eq!(counts.page_allocs, counts.page_frees);
        assert_eq!(counts.heap_allocs, counts.heap_frees);
        assert!(counts.page_allocs + counts.heap_allocs <= counts.content_checks);
        assert_eq!(counts.flushes, 2);
        assert!(pages.iter().all(Option::is_none));
        assert!(objects.iter().all(Option::is_none));
    }
}
