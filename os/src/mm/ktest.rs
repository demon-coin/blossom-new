//! 物理页与堆的独立具名验收；每例拥有资源直到停止、真实 join 与六域归还。

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

const CLASS: usize = 11;
const SLOTS: usize = 96;
const PRESSURE_CAPACITY: usize = 65536;

/// 在普通可服务 IPI 的流中归还空 slab 后排空 PCP，不改 live 对象或基线。
fn settled() -> MemoryStats {
    mm::flush_all();
    mm::shrink_all();
    mm::drain_all_pages();
    mm::stats()
}

/// 保留六域真实值，包括框架、当前 Task 与服务的常驻分配。
fn recovered(context: &mut TestContext<'_>, baseline: MemoryStats) {
    context.expect_eq(settled(), baseline);
}

/// 只在参与者静止的窗口计算可分配页数。
fn free() -> usize {
    buddy::free_pages() + pcp::cached_pages()
}

pub(crate) struct PageFixture {
    baseline: MemoryStats,
    held: [Option<OwnedPages>; 32],
}
impl Fixture for PageFixture {
    /// 所有拥有槽在 body 捕获边界之外，失败展开不会先释放 fixture。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            baseline: settled(),
            held: [const { None }; 32],
        })
    }

    /// 真实 join 已完成后消费全部页，再核六域。
    fn tear_down(mut self, context: &mut TestContext<'_>) -> TestResult {
        for slot in &mut self.held {
            drop(slot.take());
        }
        recovered(context, self.baseline);
        Ok(())
    }
}

pub(crate) struct PressureFixture {
    baseline: MemoryStats,
    pressure: Vec<OwnedPages>,
    samples: [Option<OwnedPages>; 8],
    objects: [Option<SlabObject>; SLOTS],
    heap: [Option<HeapObject>; 3],
}
impl Fixture for PressureFixture {
    /// 压力之前 fallible 预备最大句柄容量；容量归还也须恢复准备前的六域基线。
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

    /// 页、堆与 owning 表容量全部消费后，才核准备前六域。
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
    /// 真实持有全部 order10 再 order0 页，容量耗尽与物理 OOM 明确区分。
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

    /// 用原页身份选择两个非相邻样本，不制造假碎片或复制归还权。
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

/// 拥有 SLUB 唯一对象权；任何正文断言之前先登记此值。
pub(crate) struct SlabObject(NonNull<u8>);
// SAFETY: 对象唯一权随值移动，接收者在原子/锁交接后才访问；SLUB 支持异核归还。
unsafe impl Send for SlabObject {}
impl SlabObject {
    /// 取得真实 4096 class 并初始化消费者可访问的整个对象。
    fn allocate() -> Self {
        Self::try_allocate().expect("SLUB allocation failed")
    }

    /// 真实 OOM 返回 None，成功对象立即由 typed owner 接管。
    fn try_allocate() -> Option<Self> {
        let object = Self(slub::allocate(CLASS)?);
        let address = object.0.as_ptr() as usize;
        let signature = address ^ 0x5a39_26c1_78ef_04bdu64 as usize;
        // SAFETY: 刚取得独占 live 对象，整段 PAGE_SIZE 可写且首尾 usize 对齐。
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

    /// 检查仍由本值拥有的首尾签名及全部正文，不读已释放内容。
    fn check(&self) {
        let address = self.0.as_ptr() as usize;
        let signature = address ^ 0x5a39_26c1_78ef_04bdu64 as usize;
        let word = core::mem::size_of::<usize>();
        // SAFETY: 此值保留唯一 live 权且初始化已完成，无并发写者。
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
    /// 析构只归还唯一对象，不做可能再次 panic 的内容检查。
    fn drop(&mut self) {
        // SAFETY: 类型不可复制，持有原真实对象唯一权，所有借用已结束。
        unsafe { slub::deallocate(self.0) };
    }
}

pub(crate) struct SlabFixture {
    baseline: MemoryStats,
    held: [Option<SlabObject>; SLOTS],
    pages: Option<OwnedPages>,
}
impl Fixture for SlabFixture {
    /// 固定拥有槽无需堆；基线包含框架真实常驻 slab。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            baseline: settled(),
            held: [const { None }; SLOTS],
            pages: None,
        })
    }

    /// 先归还对象与页，再有限 flush/shrink/drain 核账。
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
    /// 新 owning 槽在任何身份/内容断言前取得对象。
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

    /// 借用 live 对象身份，不增加归还权。
    fn pointer(&self, index: usize) -> NonNull<u8> {
        self.held[index].as_ref().unwrap().0
    }

    /// 内容检查先于移动归还权，检查 panic 时 owning 槽仍可清理。
    fn release(&mut self, index: usize) {
        self.held[index].as_ref().unwrap().check();
        drop(self.held[index].take());
    }
}

/// 原 Layout 和唯一指针一起移动，成功 realloc 时一次替换，失败仍保原值。
pub(crate) struct HeapObject {
    pointer: NonNull<u8>,
    layout: Layout,
}
// SAFETY: 指针与原布局的唯一权整体移动，所有访问先结束，GlobalAlloc 支持异核归还。
unsafe impl Send for HeapObject {}
impl HeapObject {
    /// 经真实 GlobalAlloc 入口取得合法非零布局。
    fn allocate(layout: Layout, zeroed: bool) -> Self {
        assert!(0 < layout.size());
        // SAFETY: Layout 合法非零，MM 已初始化；返回值随后由本 owner 独占。
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

    /// 只初始化消费者请求范围，不读取 rounded 容量。
    fn fill(&mut self, byte: u8) {
        // SAFETY: 本对象独占 live 请求范围，没有其他访问者。
        unsafe { self.pointer.as_ptr().write_bytes(byte, self.layout.size()) };
    }

    /// 检查已经初始化的请求前缀，所有访问结束后才能归还。
    fn check(&self, length: usize, byte: u8) {
        assert!(length <= self.layout.size());
        for offset in 0..length {
            // SAFETY: 调用点已初始化该前缀，owner 保持 live，offset 有界。
            assert_eq!(unsafe { self.pointer.as_ptr().add(offset).read() }, byte);
        }
    }

    /// 成功时提交新指针与布局，NULL 时保留原拥有和内容。
    fn resize(&mut self, size: usize) -> bool {
        assert!(0 < size, "realloc requires a nonzero size");
        let layout = Layout::from_size_align(size, self.layout.align()).unwrap();
        // SAFETY: 当前唯一指针与原 Layout 一致，新尺寸合法非零；失败保原权。
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
    /// 使用最后成功分配的原 Layout 消费唯一权。
    fn drop(&mut self) {
        // SAFETY: 本值唯一拥有当前 live 指针，layout 仅在成功 realloc 时同步更新。
        unsafe { alloc::alloc::dealloc(self.pointer.as_ptr(), self.layout) };
    }
}

pub(crate) struct HeapFixture {
    baseline: MemoryStats,
    held: [Option<HeapObject>; 32],
    vector: Option<Vec<usize>>,
    boxed: Option<alloc::boxed::Box<[usize; 12]>>,
}
impl Fixture for HeapFixture {
    /// 正文之外保留原始堆权与标准容器。
    fn set_up(_: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
        Ok(Self {
            baseline: settled(),
            held: [const { None }; 32],
            vector: None,
            boxed: None,
        })
    }

    /// 消费真实容器 Drop 与堆权后核六域。
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
