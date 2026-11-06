//! 物理页块与全局堆对象的本核分配、跨核迁移异核释放及并发驻留压力测试。
//!
//! # 模块职责与架构定位
//! 本模块隶属于 `execution_core` 测试子树，专门检验内存管理子系统（伙伴系统、每 CPU 页缓存 PCP、SLUB 12 档对象缓存及全局堆分配器）与任务调度跨核迁移之间的协作正确性：
//! - **上游调用者**：由内核测试框架（`ktest`）调度执行，依托套件级资源 [`SuiteBaseline`] 与用例级夹具 [`Memory`] 进行双层内存账务兜底。
//! - **下游依赖**：调用 `mm::alloc_pages`、`alloc::alloc::{alloc, dealloc}`、`mm::flush_all`、`sched::test_migrate_to` 以及父模块的 [`baseline`] / [`reclaimed`]。
//!
//! # 核心设计与所有权防泄漏布局
//! 1. **异常安全的托管槽设计**：
//!    测试任务分配的所有 [`mm::OwnedPages`] 与 [`HeapObject`] 均不存放在可能发生断言失败展开的局部栈变量中，而是立即存入由夹具 [`Memory`] 持有的 [`Payload`] 固定数组槽位内。即使被测任务中途 `panic!`，夹具在 `quiesce` 等待子任务彻底退出内核栈后，仍会在 `tear_down` 析构 [`Control`] 时自动释放所有已分配的物理页与堆对象，杜绝测试失败污染全局内存池。
//!
//! 2. **跨核分配与异核释放（Cross-CPU Free）验证模型**：
//! ```text
//!   [源核 CPU_origin]                                    [目的核 CPU_target]
//!   -----------------                                    -------------------
//!   1. 分配 order-0 单页 + order-3 连续页 (8 页)
//!   2. 分配 SLUB 全部 12 档大小堆对象并填充特征字节
//!   3. sched::test_migrate_to(CPU_target) ────────────>  4. 核对 Per-CPU 基址、Task ID 与同一内核栈物理范围
//!                                                        5. 逐字节校验所有页与堆对象内容未损坏
//!                                                        6. 在目的核 Drop 释放 (触发跨核释放/远端 Slab 归还)
//!                                                        7. mm::flush_all() 同步排空各核本地缓存
//! ```
//!
//! # 锁获取顺序与并发契约
//! - **`Control::payload`（[`SleepMutex<Payload>`]）**：保护测试负载槽位与计数器。持锁期间允许任务主动调用 `sched::test_migrate_to` 跨核迁移，以及调用 `mm::flush_all` 发送跨核排空 IPI（因为 `SleepMutex` 不会关闭中断或禁用抢占）。
//! - **锁层级边界**：`Control::payload` 睡眠锁位于内存分配器内部所有自旋锁（SLUB、PCP、Buddy 自旋锁）之上；[`new_page`]、[`HeapObject`] 的分配与释放均在每次调用返回前释放了 MM 内部自旋锁，不会将自旋锁带入跨核迁移路径。

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

/// 高水位并发驻留测试（[`pressure`]）中同时持有的零阶物理页槽位数量。
const PRESSURE_PAGES: usize = 64;
/// 高水位并发驻留测试（[`pressure`]）中同时持有的全局堆对象槽位数量（覆盖 12 个 SLUB 等级各 8 轮）。
const PRESSURE_OBJECTS: usize = 96;

/// 记录单次测试运行期间各阶段的分配、释放、跨核跳转与校验次数统计。
#[derive(Default)]
struct Counts {
    /// 执行测试主体的内核任务 ID（用于确认跨核迁移前后始终为同一个任务）。
    task: usize,
    /// 测试任务在执行过程中实际驻留并完成校验过的逻辑 CPU 集合。
    visited: cpu::CpuSet,
    /// 成功完成的跨核主动迁移（Hop）总次数。
    hops: usize,
    /// 在主核（CPU 0）原地完成的分配-校验-释放轮次数。
    local_rounds: usize,
    /// 累计成功分配的物理页块（`OwnedPages`）次数。
    page_allocs: usize,
    /// 累计成功释放的物理页块次数（正常结束时必须等于 `page_allocs`）。
    page_frees: usize,
    /// 累计成功分配的全局堆对象（`HeapObject`）次数。
    heap_allocs: usize,
    /// 累计成功释放的全局堆对象次数（正常结束时必须等于 `heap_allocs`）。
    heap_frees: usize,
    /// 调用 `mm::flush_all()` 排空每 CPU 缓存的累计次数。
    flushes: usize,
    /// 在 [`pressure`] 阶段同时驻留的物理页总数（单位：4 KiB 页）。
    pressure_pages: usize,
    /// 在 [`pressure`] 阶段同时驻留的全局堆对象总数。
    pressure_objects: usize,
    /// 完成全字节内容比对的物理页块与堆对象累计检查次数。
    content_checks: usize,
    /// 完成当前 CPU 编号与 Per-CPU 硬件基址一致性校验的累计次数。
    cpu_checks: usize,
    /// 完成任务局部栈变量虚拟/物理地址范围不变性校验的累计次数。
    stack_checks: usize,
}

/// 任务内核栈帧的不变性见证快照，用于验证任务跨核迁移后仍运行在同一块物理栈上。
struct StackWitness {
    /// 任务入口栈帧上哨兵局部变量 `local` 的虚拟地址（字节）。
    address: usize,
    /// 任务入口时查询到的当前任务内核栈物理起始地址与字节长度 `(phys_start, byte_len)`。
    physical: (usize, usize),
}

/// 携带原始内存布局与特征填充字节的全局堆分配 RAII 封装，用于在析构时向全局分配器归还内存。
struct HeapObject {
    /// `alloc::alloc::alloc` 返回的非空堆对象起始指针。
    pointer: NonNull<u8>,
    /// 分配该对象时使用的精确大小与对齐布局（`dealloc` 时必须原样传回）。
    layout: Layout,
    /// 分配后写入整个对象字节区间的预期特征值，供释放前校验内容是否被篡改。
    pattern: u8,
}
// SAFETY:
// 1. `HeapObject` 独占持有全局堆分配器返回的一块独立内存，不可克隆且不向外部暴露可逃逸的裸指针或引用。
// 2. 内核全局堆分配器（SLUB 后端）支持在一个 CPU/任务上分配并在另一个 CPU/任务上按原 `Layout` 调用 `dealloc` 释放。
// 3. 对底层内存的读写始终受外部 `Control::payload` 睡眠互斥锁串行化保护，不存在跨线程并发数据竞争。
unsafe impl Send for HeapObject {}

impl HeapObject {
    /// 按指定字节大小（8 字节对齐）从全局堆分配对象，并将全部字节填充为 `pattern`。
    fn new(size: usize, pattern: u8) -> Self {
        let layout = Layout::from_size_align(size, 8).expect("X24 heap layout");
        // SAFETY:
        // 1. 调用方传入的 `size` 取自 `SLUB_SIZES` 常量表，均大于 0，且上方 `Layout::from_size_align` 已校验大小与 8 字节对齐合法。
        // 2. 当前运行在内存管理已初始化的普通可调度任务中，未持有任何分配器自旋锁或关中断守卫。
        let pointer = NonNull::new(unsafe { alloc(layout) }).expect("X24 heap allocation failed");
        // SAFETY:
        // 1. 上方 `NonNull::new(...).expect(...)` 已确认 `alloc` 成功返回了至少包含 `size` 个连续可写字节的独占堆内存。
        // 2. 该指针刚分配完成尚未共享给任何其他执行流，按字节写入 `size` 长度不会越界或产生别名冲突。
        unsafe { pointer.as_ptr().write_bytes(pattern, size) };
        Self {
            pointer,
            layout,
            pattern,
        }
    }

    /// 逐字节核对堆对象内容是否仍全部等于初始填充的 `pattern`。
    fn check(&self) {
        // SAFETY:
        // 1. `HeapObject::new` 在构造时已将 `[pointer, pointer + layout.size())` 全部字节初始化为 `pattern`，且 `Drop` 尚未执行，内存保持有效。
        // 2. 调用方持有 `Control::payload` 互斥锁独占访问本对象，期间无任何并发写者，构造的临时只读切片在本函数返回前即结束生命周期。
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
    /// 按照构造时记录的原始 `Layout` 将独占堆内存归还给全局分配器。
    fn drop(&mut self) {
        // SAFETY:
        // 1. `self.pointer` 由 `HeapObject::new` 调用全局 `alloc(self.layout)` 获得，且 `self.layout` 自构造以来从未修改。
        // 2. `&mut self` 保证所有临时只读切片借用均已结束，且 `Drop` 在对象生命周期结束时仅执行一次，不会重复释放。
        unsafe { dealloc(self.pointer.as_ptr(), self.layout) };
    }
}

/// 从物理页分配器申请 `2^order` 页连续内存，校验阶数与自然对齐后将整块内存填充为 `pattern`。
fn new_page(order: usize, pattern: u8, counts: &mut Counts) -> mm::OwnedPages {
    let mut pages = mm::alloc_pages(order, false).expect("X24 page allocation failed");
    assert_eq!(pages.order(), order);
    assert_eq!(pages.phys_addr() % pages.len(), 0);
    let len = pages.len();
    // SAFETY:
    // 1. `mm::alloc_pages` 返回当前任务独占持有的 `OwnedPages`，其对应的直接映射虚拟地址区间 `[as_mut_ptr(), as_mut_ptr() + len)` 永久有效且可写。
    // 2. `len` 等于 `pages.len()` 精确字节数，直接调用 `write_bytes` 初始化整段内存不会越界，也不会在初始化前构造未初始化内存的引用。
    unsafe { pages.as_mut_ptr().cast::<u8>().write_bytes(pattern, len) };
    counts.page_allocs += 1;
    pages
}

/// 校验当前任务确实运行在目标逻辑核 `target` 上，且硬件 Per-CPU 指针、任务 ID 以及当前内核栈帧物理地址与初始见证一致。
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

/// 在空的托管槽位中分配一轮测试对象：包含 1 个零阶单页、1 个 3 阶（8 页）连续块，以及覆盖全部 12 档 SLUB 大小等级的堆对象。
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

/// 逐字节校验托管槽中所有物理页块与堆对象的内容完整性，随后逐一取出并释放归还给分配器。
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
        // SAFETY:
        // 1. 该 `pages` 由 `new_page` 分配并已用 `write_bytes` 初始化全部 `len` 个字节，且已通过 `slot.take()` 移交当前局部变量独占持有。
        // 2. 临时只读切片仅用于紧随其后的 `all` 断言，在下方 `drop(pages)` 释放物理页之前即结束借用。
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

/// 在普通可调度任务上下文中调用 `mm::flush_all()` 同步排空所有 CPU 的本地 SLUB 缓存，并递增排空计数。
fn flush(counts: &mut Counts) {
    mm::flush_all();
    counts.flushes += 1;
}

/// 执行高水位并发驻留压力测试：同时填满 64 个单页槽位与 96 个多档位堆对象槽位，跨 `flush_all` 验证内容完好后全部释放并再次排空。
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

/// 由睡眠互斥锁保护的测试对象托管仓库与统计账本，放在测试任务栈外以保证异常展开时内存仍能被夹具回收。
struct Payload {
    /// 托管物理页块所有权的固定容量槽位数组。
    pages: [Option<mm::OwnedPages>; PRESSURE_PAGES],
    /// 托管全局堆对象所有权的固定容量槽位数组。
    objects: [Option<HeapObject>; PRESSURE_OBJECTS],
    /// 本用例执行期间的累计分配、释放、迁核与校验计数器。
    counts: Counts,
}

/// 父夹具与派生测试子任务之间共享的控制块。
struct Control {
    /// 受睡眠互斥锁保护的对象托管槽与统计计数。
    payload: SleepMutex<Payload>,
    /// 协作式停止标志，夹具进入 `quiesce` 时置为 `true` 通知迁核循环提前退出。
    stop: AtomicBool,
}

/// 内存分配与跨核释放测试夹具，负责记录用例前基线、托管在途分配对象并在析构时核对全系统内存收敛。
struct Memory {
    /// 用例启动前采集的全系统静止态内存与工作队列元数据基线。
    initial: Baseline,
    /// 跨任务共享的托管仓库与停止标志。
    state: Arc<Control>,
    /// 跨核迁移用例（`migrating_owners`）派生的子任务句柄；本核原地用例为 `None`。
    handle: Option<ChildHandle>,
}

impl Fixture<SuiteBaseline> for Memory {
    /// 采集用例级初始内存基线，并在堆上预先建立空的托管槽位控制块，确保后续子任务发生异常时已分配对象不会泄漏。
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

    /// 发出协作停止信号，并等待派生子任务（若有）彻底执行完毕并脱离其内核栈，防止在子任务退出前过早回收资源。
    fn quiesce(&mut self, _: &mut TestContext<'_>) -> TestResult {
        self.state.stop.store(true, Ordering::Release);
        crate::ktest::execution_core::until(
            || {
                self.handle
                    .as_ref()
                    .is_none_or(|handle| handle.try_join().is_some())
            },
            "memory consumer did not reach original off-stack completion",
        )
    }

    /// 释放子任务句柄与共享控制块（连带归还其中可能残留的页块和堆对象），最后核对全系统内存与元数据统计恢复至用例前基线。
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

/// 执行内存分配、内容校验与可选跨核迁移异核释放的主测试流程。
///
/// 调用上下文：持有 [`Control::payload`] 睡眠互斥锁的普通可调度内核任务。
fn consume(fixture: &mut Payload, migrate: bool, stop: &AtomicBool) {
    // Step 1: 在当前任务栈上建立局部变量见证，迁移至主核（CPU 0）并完成初始执行环境与栈范围校验。
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
    // Step 2: 在 CPU 0 上原地执行 2 轮“分配 -> 校验并释放 -> 排空缓存 -> 核对执行环境”。
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

    // Step 3: 若启用跨核模式，携带源核分配的物理页与 12 类堆对象依次环游所有在线 CPU 两整圈，在目的核校验内容后执行异核释放与缓存排空。
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

    // Step 4: 汇总断言所有托管槽位均已清空，且任务身份、访问核集合、迁核跳数及分配/释放/校验账目完全吻合。
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

#[expect(
    clippy::module_inception,
    reason = "具名 suite 的 memory 层级属于现有测试目录，外层文件模块承载同组原页 fixture"
)]
#[::ktest::suite(fixture = Memory, suite_resources = crate::ktest::execution_services::SuiteBaseline, tags("execution", "mm"), timeout_ms = 20000)]
pub(crate) mod memory {
    use super::*;
    /// 验证单核（CPU 0）原地连续两轮分配、内容读写校验与释放物理页（零阶与 3 阶）及全部 12 档 SLUB 堆对象的正确性。
    ///
    /// - **测试目标**：确认在不涉及跨核迁移的基础路径上，伙伴系统各阶页块与全部 SLUB 大小等级的对象分配、内容保持、释放及缓存排空后内存账闭合。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：夹具采集系统内存基线并初始化空托管槽；
    ///   2. When：当前任务锁定托管仓库，迁移至 CPU 0 连续执行两轮 [`allocate_round`]、[`check_and_release`] 与 [`flush`]；
    ///   3. Then：每轮对象字节内容完好无损，分配与释放次数相等，且夹具 `tear_down` 时全系统内存统计完全恢复至基线。
    #[test]
    fn local_owners(_: &mut TestContext<'_>, fixture: &mut Memory) {
        consume(
            &mut fixture.state.payload.lock(),
            false,
            &fixture.state.stop,
        );
    }

    /// 验证同一个内核任务携带已分配的物理页块与 12 档全局堆对象跨越所有在线 CPU 往返两轮，并在目的核完成异核释放与缓存排空。
    ///
    /// - **测试目标**：确认任务跨核迁移后内核栈物理区间、栈上局部变量及堆/页数据内容保持不变，且伙伴系统 PCP 与 SLUB 分配器支持在源核分配、在任意远端核安全释放并正确收敛内存账。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：系统至少有 2 个在线 CPU，派生独立的测试子任务并锁定托管仓库；
    ///   2. When：子任务先在 CPU 0 完成 2 轮本地分配释放，随后循环 `2 * cpu::count()` 跳：每次在源核分配零阶页、3 阶页及 12 档堆对象，调用 `sched::test_migrate_to` 迁移至下一个目标核，校验 Per-CPU 状态、栈见证与内存内容后在目标核释放并执行 `flush_all`；
    ///   3. Then：子任务访问遍及全部在线 CPU 并返回预期退出码 `24`，所有页与堆对象无一损坏或泄漏，`tear_down` 对账确认全系统内存完全恢复至初始基线。
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

    /// 验证 64 个物理页与 96 个多档位全局堆对象同时存活驻留、跨越缓存排空操作后的内容无串扰与全量回收。
    ///
    /// - **测试目标**：确认在高并发存活对象数量下，各物理页与各档 SLUB Slab 对象之间不存在地址重叠或缓存排空（`flush_all`）误回收仍在用对象的问题。
    /// - **场景步骤（Given-When-Then）**：
    ///   1. Given：锁定夹具托管仓库，确认 64 个页槽位与 96 个堆对象槽位均为空；
    ///   2. When：一次性填满全部 64 个零阶物理页与 96 个轮询覆盖 12 档大小的堆对象（填充 `0xd4`），在全部对象仍存活时执行第一次 `mm::flush_all()`，随后逐字节核对全部 160 个内存块并逐一释放，最后执行第二次 `mm::flush_all()`；
    ///   3. Then：所有存活对象跨越排空后字节内容保持 `0xd4` 无误，分配与释放计数严格相等，且夹具析构时内存账完全闭合。
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
