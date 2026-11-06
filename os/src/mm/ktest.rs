//! 物理内存与内核堆子系统（`mm`）的集成测试总入口与通用测试夹具（Fixtures）定义。
//!
//! ## 模块职责与架构定位
//! 本模块作为 `mm` 子系统的 `ktest` 测试根模块，组织管理伙伴分配器（Buddy）、每 CPU 页缓存（PCP）、
//! SLUB 对象分配器、全局堆（GlobalAlloc）、时钟中断（IRQ）抢占、多核并发（SMP）以及异常展开（Unwind）等子测试模块。
//! 同时提供 `PageFixture`、`PressureFixture`、`SlabFixture` 和 `HeapFixture` 四类资源托管夹具，
//! 确保所有测试分配的物理页、SLUB 对象与堆内存无论在正常结束还是 `panic!` 栈展开时都能完整归还，并校验系统内存统计完全恢复至初始基线。
//!
//! ## 核心设计与生命周期模型
//!
//! ### 1. 夹具资源托管与六域账务核对流程
//! ```text
//! [set_up]
//!    |-- settled(): flush_all() -> shrink_all() -> drain_all_pages()
//!    +-- 记录 baseline: MemoryStats (present, reserved, metadata, free, slab, pcp 等)
//!          |
//!          v
//! [测试函数执行 (Test Body)]
//!    |-- 所有分配出的 OwnedPages / SlabObject / HeapObject 存入 Fixture 槽位
//!    +-- 即使测试中途触发 panic!，栈展开也不会丢失 Fixture 中的资源句柄
//!          |
//!          v
//! [tear_down]
//!    |-- 依次 Drop 所有槽位及预留容量的容器 (如 PressureFixture::pressure)
//!    |-- settled(): 刷回远端释放队列、收缩空 Slab、排空所有 CPU 的 PCP 缓存
//!    +-- 断言当前 MemoryStats == baseline (零内存泄漏)
//! ```
//!
//! ### 2. `SlabObject` 内存完整性签名布局（4096 字节等级）
//! ```text
//! +-----------------------+-----------------------------------+-----------------------+
//! | 首部签名 (usize)      | 填充正文 (0xa7, 4096 - 2*usize)   | 尾部反码签名 (!sig)   |
//! | addr ^ 0x5a3926c1...  |                                   | !(addr ^ 0x5a3926...) |
//! +-----------------------+-----------------------------------+-----------------------+
//! 0                   word                             4096 - word                 4096
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `settled()` 会触发跨核处理器间中断（IPI）或刷回操作，必须在开中断、可响应 IPI 的普通任务上下文中调用。
//! - `SlabObject` 与 `HeapObject` 封装了独占的分配指针（`NonNull<u8>`），跨线程传递时通过移动所有权（`Send`）保证同一时刻只有一个执行流读写或释放该对象。

mod algorithms;
mod fixed;
mod heap;
mod irq;
mod lifecycle;
mod pages;
mod slabs;
mod smp;

pub(crate) use irq::{timer_irq, AllocationWindow};

use crate::mm::{self, buddy, pcp, slub, MemoryStats, OwnedPages, PAGE_SIZE};
use alloc::vec::Vec;
use core::alloc::Layout;
use core::hint::black_box;
use core::ptr::NonNull;
use ktest::{Fixture, TestContext, TestError, TestResult};

/// 测试使用的 SLUB 大小等级索引（对应 4096 字节对象）。
const CLASS: usize = 11;
/// `SlabFixture` 与 `PressureFixture` 中预留的 SLUB 对象槽位数量。
const SLOTS: usize = 96;
/// `PressureFixture` 为模拟物理内存耗尽（OOM）预先分配的 `OwnedPages` 句柄向量容量上限。
const PRESSURE_CAPACITY: usize = 65536;

/// 刷回所有远端 SLUB 释放链表、回收空 Slab 页并排空全部 CPU 的本地页缓存（PCP），返回稳定后的系统内存统计快照。
///
/// 调用上下文：必须在普通任务上下文（允许处理跨核同步）且测试工作线程均已静止时调用。
fn settled() -> MemoryStats {
    mm::flush_all();
    mm::shrink_all();
    mm::drain_all_pages();
    mm::stats()
}

/// 校验当前稳定后的系统内存统计数据是否与测试开始前记录的 `baseline` 完全一致（包含内核常驻分配与当前测试任务栈）。
fn recovered(context: &mut TestContext<'_>, baseline: MemoryStats) {
    context.expect_eq(settled(), baseline);
}

/// 计算当前伙伴系统（Buddy）空闲页数与所有 CPU 本地缓存（PCP）页数之和，用于静止窗口下的可用页断言。
fn free() -> usize {
    buddy::free_pages() + pcp::cached_pages()
}

/// 物理页分配测试夹具，提供固定数量的 `OwnedPages` 托管槽位与内存统计基线校验。
pub(crate) struct PageFixture {
    /// 测试开始前通过 `settled()` 记录的系统内存统计基线（页粒度）。
    baseline: MemoryStats,
    /// 托管被测物理页块所有权的固定槽位数组，确保测试 `panic!` 时仍能由 `tear_down` 统一释放。
    held: [Option<OwnedPages>; 32],
}
impl Fixture for PageFixture {
    /// 初始化夹具：排空缓存并记录内存统计基线，将全部 32 个物理页托管槽位置空。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            baseline: settled(),
            held: [const { None }; 32],
        })
    }

    /// 清理夹具：释放槽位中持有的全部物理页块，并断言系统内存统计恢复至 `baseline`。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        for slot in &mut self.held {
            drop(slot.take());
        }
        recovered(context, self.baseline);
        Ok(())
    }
}

/// 内存压力与 OOM 测试夹具，预先保留大容量句柄数组，用于在无额外堆分配干扰的前提下耗尽系统物理页。
pub(crate) struct PressureFixture {
    /// 预留 `pressure` 容器容量之前记录的系统内存统计基线。
    baseline: MemoryStats,
    /// 预分配了 `PRESSURE_CAPACITY` 容量的物理页句柄列表，用于持续分配并持有物理页直至系统 OOM。
    pressure: Vec<OwnedPages>,
    /// 用于碎片化与非相邻页挑选测试的 8 个样本页槽位。
    samples: [Option<OwnedPages>; 8],
    /// 压力测试期间托管 SLUB 对象的固定槽位数组。
    objects: [Option<SlabObject>; SLOTS],
    /// 压力测试期间托管全局堆对象的固定槽位数组。
    heap: [Option<HeapObject>; 3],
}
impl Fixture for PressureFixture {
    /// 初始化夹具：先获取内存基线，随后以可失败方式（`try_reserve_exact`）预分配句柄向量容量。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        let baseline = settled();
        let mut pressure = Vec::new();
        pressure
            .try_reserve_exact(PRESSURE_CAPACITY)
            .map_err(|_| TestError("NoMemory: pressure handles"))?;
        Ok(Self {
            baseline,
            pressure,
            samples: [const { None }; 8],
            objects: [const { None }; SLOTS],
            heap: [const { None }; 3],
        })
    }

    /// 清理夹具：依次释放堆对象、SLUB 对象、样本页、压力页以及 `pressure` 向量自身的堆缓冲区，最后校验内存统计恢复至准备前基线。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        for slot in &mut self.heap {
            drop(slot.take());
        }
        for slot in &mut self.objects {
            drop(slot.take());
        }
        for slot in &mut self.samples {
            drop(slot.take());
        }

        self.pressure.clear();
        let baseline = self.baseline;
        drop(self);
        recovered(context, baseline);
        Ok(())
    }
}
impl PressureFixture {
    /// 排空 PCP 后先按最大阶（`MAX_ORDER`）再按单页（Order 0）持续分配物理页直至返回 `None`，使系统进入空闲页为 0 的真实 OOM 状态。
    fn exhaust(&mut self) {
        mm::drain_all_pages();
        for order in [buddy::MAX_ORDER, 0] {
            loop {
                assert!(
                    self.pressure.len() < self.pressure.capacity(),
                    "pressure handle capacity exhausted"
                );
                let Some(pages) = black_box(mm::alloc_pages(order, false)) else {
                    break;
                };
                self.pressure.push(pages);
            }
        }
        assert_eq!(free(), 0, "real pressure did not exhaust pages");
    }

    /// 分配 8 个单页样本并从中挑选物理地址不相邻（间距大于 `PAGE_SIZE`）的两个槽位索引，用于验证非伙伴页不会误合并。
    fn nonadjacent(&mut self) -> (usize, usize) {
        for slot in &mut self.samples {
            *slot = Some(mm::alloc_pages(0, false).expect("fragment sample allocation failed"));
        }

        for first in 0..self.samples.len() {
            for second in first + 1..self.samples.len() {
                if PAGE_SIZE
                    < self.samples[first]
                        .as_ref()
                        .unwrap()
                        .phys_addr()
                        .abs_diff(self.samples[second].as_ref().unwrap().phys_addr())
                {
                    return (first, second);
                }
            }
        }
        panic!("fragment samples contain no nonadjacent pair")
    }
}

/// 独占持有单个 SLUB 4096 字节等级对象的测试包装类型（元组字段 `0` 为指向 `slub::allocate(CLASS)` 分配对象的非空指针），自带首尾地址混淆签名与正文填充校验。
pub(crate) struct SlabObject(NonNull<u8>);
// SAFETY:
// 1. `SlabObject` 独占持有底层 SLUB 对象的唯一所有权，且不实现 `Clone` 或 `Copy`；
// 2. 跨线程移动后仅由接收方访问，且底层 SLUB 分配器原生支持跨 CPU 释放（`slub::deallocate`）。
unsafe impl Send for SlabObject {}
impl SlabObject {
    /// 分配一个 `CLASS`（4096 字节）等级的 SLUB 对象并写入测试签名，分配失败时直接触发 `panic!`。
    fn allocate() -> Self {
        Self::try_allocate().expect("SLUB allocation failed")
    }

    /// 尝试分配一个 `CLASS`（4096 字节）等级的 SLUB 对象；若成功则用 `0xa7` 填充正文并在首尾写入与地址相关的校验签名，OOM 时返回 `None`。
    fn try_allocate() -> Option<Self> {
        let object = Self(slub::allocate(CLASS)?);
        let address = object.0.as_ptr() as usize;
        let signature = address ^ 0x5a39_26c1_78ef_04bdu64 as usize;
        // SAFETY:
        // 1. `slub::allocate(CLASS)` 成功返回刚分配的独占对象指针，对应大小为 `PAGE_SIZE`（4096 字节）且满足页对齐（自然满足 `usize` 对齐）；
        // 2. 首字偏移 `0` 与尾字偏移 `PAGE_SIZE - size_of::<usize>()` 均严格落在 `[0, PAGE_SIZE)` 范围内，无越界或并发竞争。
        unsafe {
            object.0.as_ptr().write_bytes(0xa7, PAGE_SIZE);
            object.0.as_ptr().cast::<usize>().write(signature);
            object
                .0
                .as_ptr()
                .add(PAGE_SIZE - core::mem::size_of::<usize>())
                .cast::<usize>()
                .write(!signature);
        }
        Some(object)
    }

    /// 校验当前 SLUB 对象的首字签名、尾字反码签名以及中间全部 `0xa7` 填充字节是否完好无损。
    fn check(&self) {
        let address = self.0.as_ptr() as usize;
        let signature = address ^ 0x5a39_26c1_78ef_04bdu64 as usize;
        let word = core::mem::size_of::<usize>();
        // SAFETY:
        // 1. `&self` 保证当前 `SlabObject` 仍处于存活期且在 `try_allocate` 中已完成全部 `PAGE_SIZE` 字节的初始化；
        // 2. 首尾按 `usize` 读取及中间按字节读取的偏移均位于 `[0, PAGE_SIZE)` 内，且无并发写者。
        unsafe {
            assert_eq!(self.0.as_ptr().cast::<usize>().read(), signature);
            assert_eq!(
                self.0.as_ptr().add(PAGE_SIZE - word).cast::<usize>().read(),
                !signature
            );
            for offset in word..PAGE_SIZE - word {
                assert_eq!(self.0.as_ptr().add(offset).read(), 0xa7);
            }
        }
    }
}
impl Drop for SlabObject {
    /// 析构时将独占持有的对象归还给 SLUB 分配器（不在 Drop 中执行断言检查，避免栈展开期间触发二次 panic）。
    fn drop(&mut self) {
        // SAFETY:
        // 1. `SlabObject` 独占持有 `self.0` 指向的 `CLASS` 等级 SLUB 对象；
        // 2. 在 `Drop::drop` 中归还后不会再有任何代码访问该指针。
        unsafe { slub::deallocate(self.0) };
    }
}

/// SLUB 分配器测试夹具，托管固定数量的 `SlabObject` 槽位与一个可选的物理页块。
pub(crate) struct SlabFixture {
    /// 测试开始前稳定化后的系统内存统计基线。
    baseline: MemoryStats,
    /// 托管被测 `SlabObject` 的固定大小槽位数组。
    held: [Option<SlabObject>; SLOTS],
    /// 托管辅助测试物理页块的可选槽位。
    pages: Option<OwnedPages>,
}
impl Fixture for SlabFixture {
    /// 初始化夹具：稳定内存状态并记录统计基线，清空所有对象与页槽位。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            baseline: settled(),
            held: [const { None }; SLOTS],
            pages: None,
        })
    }

    /// 清理夹具：释放全部持有的 `SlabObject` 与物理页，刷回并收缩 SLUB 缓存后校验内存统计恢复至基线。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        for slot in &mut self.held {
            drop(slot.take());
        }
        drop(self.pages.take());
        recovered(context, self.baseline);
        Ok(())
    }
}
impl SlabFixture {
    /// 在指定空槽位 `index` 中分配一个新的 `SlabObject`，并断言其地址与当前夹具中所有已存活对象的地址均不冲突。
    fn allocate(&mut self, index: usize) {
        assert!(self.held[index].is_none());
        self.held[index] = Some(SlabObject::allocate());
        let address = self.pointer(index).as_ptr() as usize;
        for (other, object) in self.held.iter().enumerate() {
            if other != index {
                if let Some(object) = object {
                    assert_ne!(object.0.as_ptr() as usize, address);
                }
            }
        }
    }

    /// 获取指定槽位 `index` 中存活 `SlabObject` 的底层非空指针（仅读取地址，不转移所有权）。
    fn pointer(&self, index: usize) -> NonNull<u8> {
        self.held[index].as_ref().unwrap().0
    }

    /// 先校验指定槽位 `index` 中 `SlabObject` 的内容签名，校验通过后将其从槽位取出并释放（若校验 panic，对象仍留在槽位中由 `tear_down` 回收）。
    fn release(&mut self, index: usize) {
        self.held[index].as_ref().unwrap().check();
        drop(self.held[index].take());
    }
}

/// 独占持有全局堆（`GlobalAlloc`）内存块及其对应 `Layout` 的测试包装类型，支持原地或搬迁 `realloc`。
pub(crate) struct HeapObject {
    /// 指向由 `alloc` / `alloc_zeroed` / `realloc` 返回的堆内存块非空指针。
    pointer: NonNull<u8>,
    /// 当前内存块生效的分配布局（大小与对齐），在 `resize` 成功时同步更新并在 `Drop` 时用于 `dealloc`。
    layout: Layout,
}
// SAFETY:
// 1. `HeapObject` 独占持有 `pointer` 指向的堆内存块，不实现 `Clone` 或 `Copy`；
// 2. 全局堆分配器（`GlobalAlloc`）支持跨 CPU 线程安全地释放内存。
unsafe impl Send for HeapObject {}
impl HeapObject {
    /// 通过全局堆分配器（`alloc` 或 `alloc_zeroed`）分配符合非零 `layout` 要求的内存块并接管所有权。
    fn allocate(layout: Layout, zeroed: bool) -> Self {
        assert!(0 < layout.size());
        // SAFETY:
        // 1. 前置 `assert!(0 < layout.size())` 保证了传入 `alloc` / `alloc_zeroed` 的 `Layout` 大小非零，符合 `GlobalAlloc` 安全前置条件；
        // 2. 内核堆分配器在引导阶段已完成初始化，返回的非空指针随后由当前 `HeapObject` 独占管理。
        let pointer = unsafe {
            if zeroed {
                alloc::alloc::alloc_zeroed(black_box(layout))
            } else {
                alloc::alloc::alloc(black_box(layout))
            }
        };
        Self {
            pointer: NonNull::new(pointer).expect("heap allocation failed"),
            layout,
        }
    }

    /// 将当前堆内存块中调用方请求的 `[0, self.layout.size())` 范围全部填充为指定字节 `byte`。
    fn fill(&mut self, byte: u8) {
        // SAFETY:
        // 1. `&mut self` 保证当前 `HeapObject` 对 `[pointer, pointer + layout.size())` 范围拥有独占写权限；
        // 2. `self.layout.size()` 即为向全局分配器申请的合法字节数，写入不会越界。
        unsafe { self.pointer.as_ptr().write_bytes(byte, self.layout.size()) };
    }

    /// 校验当前堆内存块前 `length` 个字节是否全部等于预期值 `byte`。
    fn check(&self, length: usize, byte: u8) {
        assert!(length <= self.layout.size());
        for offset in 0..length {
            // SAFETY:
            // 1. 前置 `assert!(length <= self.layout.size())` 保证了 `offset < length <= self.layout.size()`，指针偏移不越界；
            // 2. 调用方在调用 `check` 前已通过 `allocate(..., true)` 或 `fill` 初始化了该前缀，且 `&self` 期间无并发修改。
            assert_eq!(unsafe { self.pointer.as_ptr().add(offset).read() }, byte);
        }
    }

    /// 调用 `alloc::alloc::realloc` 将当前堆内存块调整为新的非零字节数 `size`（保持原对齐）；成功时更新内部指针与 `layout` 并返回 `true`，失败时保留原内存块并返回 `false`。
    fn resize(&mut self, size: usize) -> bool {
        assert!(0 < size, "realloc requires a nonzero size");
        let layout = Layout::from_size_align(size, self.layout.align()).unwrap();
        // SAFETY:
        // 1. `self.pointer` 是由全局分配器分配且尚未释放的有效指针，`self.layout` 与最后一次成功分配/扩容的布局完全一致；
        // 2. 前置断言与 `Layout::from_size_align(...).unwrap()` 保证了新 `size > 0` 且舍入后不会发生整数溢出；若返回空指针，原内存块保持有效并继续由 `self` 持有。
        let pointer = black_box(unsafe {
            alloc::alloc::realloc(black_box(self.pointer.as_ptr()), self.layout, size)
        });
        if let Some(pointer) = NonNull::new(pointer) {
            self.pointer = pointer;
            self.layout = layout;
            true
        } else {
            false
        }
    }
}
impl Drop for HeapObject {
    /// 析构时使用当前记录的 `self.layout` 将独占堆内存块归还给全局分配器。
    fn drop(&mut self) {
        // SAFETY:
        // 1. `self.pointer` 由当前 `HeapObject` 独占持有，且 `self.layout` 始终与最后一次成功分配或 `realloc` 的布局严格匹配；
        // 2. 在 `Drop::drop` 执行后该对象生命周期结束，不会再发生悬垂访问。
        unsafe { alloc::alloc::dealloc(self.pointer.as_ptr(), self.layout) };
    }
}

/// 全局堆（`GlobalAlloc`）测试夹具，托管固定数量的 `HeapObject` 槽位以及标准库 `Vec` / `Box` 容器。
pub(crate) struct HeapFixture {
    /// 测试开始前稳定化后的系统内存统计基线。
    baseline: MemoryStats,
    /// 托管被测 `HeapObject` 的 32 个固定槽位。
    held: [Option<HeapObject>; 32],
    /// 托管被测 `Vec<usize>` 动态数组的槽位，用于验证标准库容器扩容与回收。
    vector: Option<Vec<usize>>,
    /// 托管被测 `Box<[usize; 12]>` 装箱数组的槽位。
    boxed: Option<alloc::boxed::Box<[usize; 12]>>,
}
impl Fixture for HeapFixture {
    /// 初始化夹具：稳定内存状态并记录统计基线，清空所有堆对象与容器槽位。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            baseline: settled(),
            held: [const { None }; 32],
            vector: None,
            boxed: None,
        })
    }

    /// 清理夹具：释放全部 `HeapObject`、`Vec` 与 `Box`，排空缓存后校验系统内存统计完全恢复至基线。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        for slot in &mut self.held {
            drop(slot.take());
        }
        drop(self.vector.take());
        drop(self.boxed.take());
        recovered(context, self.baseline);
        Ok(())
    }
}
