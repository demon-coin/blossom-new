//! 异常栈展开（Unwind）期间内存测试夹具（Fixtures）的资源保活与自动回收验收测试。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `mm::ktest` 下，专门验证 `PageFixture`、`SlabFixture`、`HeapFixture` 与 `PressureFixture`
//! 在测试用例函数触发预期 `panic!`（标注 `#[should_panic]`）时的异常安全性（Exception Safety）。
//! 确保测试函数栈展开不会导致物理页、SLUB 对象、全局堆内存或全系统 OOM 压力页泄漏，且夹具 `tear_down` 仍能在捕获 panic 后将系统内存统计完全恢复至初始基线。
//!
//! ## 核心机制：预期 Panic 下的夹具生命周期
//! ```text
//! [ktest 框架]
//!    |-- 1. Fixture::set_up() -> 记录 baseline，初始化托管槽位 (位于 catch_unwind 边界之外)
//!    |-- 2. catch_unwind(|| test_body(&mut fixture))
//!    |        |-- 分配物理页 / SLUB / Heap / OOM 压力页并存入 fixture
//!    |        +-- 触发 panic!("... body unwind") -> 栈展开回退至 catch_unwind
//!    |-- 3. Fixture::tear_down(fixture) -> 释放所有槽位中的资源，断言 MemoryStats == baseline
//!    +-- 4. 校验捕获的 panic 消息匹配 expected，标记测试通过
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - 所有生命周期展开测试均在单任务上下文中执行；触发 `panic!` 前已释放所有局部临时锁，确保 `tear_down` 可安全回收资源。

use crate::mm::ktest::{HeapFixture, HeapObject, PageFixture, PressureFixture, SlabFixture};
use crate::mm::{self, PAGE_SIZE};
use core::alloc::Layout;
use ktest::TestContext;

#[ktest::suite(fixture = PageFixture, tags("mm", "lifecycle"), timeout_ms = 30000)]
mod pages {
    use super::*;
    /// 测试目标：验证物理页测试函数触发 `panic!` 栈展开时，已存入 `PageFixture` 的高阶物理页块（Order 1）不会泄漏并能按原阶安全归还。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：由 `PageFixture::set_up` 记录初始内存统计基线。
    /// 2. When：分配一个 Order 1（2 页）的清零物理页块存入 `f.held[0]`，校验长度后主动触发 `panic!("owned page body unwind")`。
    /// 3. Then：栈展开被框架捕获后执行 `PageFixture::tear_down`，释放 `f.held[0]` 中的 2 页物理块，系统内存统计完全恢复至基线。
    #[test]
    #[should_panic(expected = "owned page body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut PageFixture) {
        f.held[0] = Some(mm::alloc_pages(1, true).unwrap());
        assert_eq!(f.held[0].as_ref().unwrap().len(), PAGE_SIZE * 2);
        panic!("owned page body unwind");
    }
}

#[ktest::suite(fixture = SlabFixture, tags("mm", "lifecycle"), timeout_ms = 30000)]
mod objects {
    use super::*;
    /// 测试目标：验证 SLUB 测试函数触发 `panic!` 栈展开时，已存入 `SlabFixture` 的 `SlabObject` 保持有效并由夹具统一归还与收缩。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：由 `SlabFixture::set_up` 记录初始内存统计基线。
    /// 2. When：在槽位 `0` 分配一个 4096 字节等级的 `SlabObject` 并校验其内容签名，随后主动触发 `panic!("slab body unwind")`。
    /// 3. Then：栈展开后执行 `SlabFixture::tear_down`，归还该 SLUB 对象并收缩空 Slab 页，系统内存统计完全恢复至基线。
    #[test]
    #[should_panic(expected = "slab body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut SlabFixture) {
        f.allocate(0);
        f.held[0].as_ref().unwrap().check();
        panic!("slab body unwind");
    }
}

#[ktest::suite(fixture = HeapFixture, tags("mm", "lifecycle"), timeout_ms = 30000)]
mod heap {
    use super::*;
    /// 测试目标：验证全局堆测试函数触发 `panic!` 栈展开时，已存入 `HeapFixture` 的大尺寸对齐堆内存块及其 `Layout` 不会丢失并能完整释放。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：由 `HeapFixture::set_up` 记录初始内存统计基线。
    /// 2. When：分配一个大小为 8193 字节、对齐为 64 字节的 `HeapObject` 存入 `f.held[0]` 并填充 `0x6d`，随后主动触发 `panic!("heap body unwind")`。
    /// 3. Then：栈展开后执行 `HeapFixture::tear_down`，按原 `Layout` 调用 `dealloc` 释放堆内存，系统内存统计完全恢复至基线。
    #[test]
    #[should_panic(expected = "heap body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut HeapFixture) {
        f.held[0] = Some(HeapObject::allocate(
            Layout::from_size_align(8193, 64).unwrap(),
            false,
        ));
        f.held[0].as_mut().unwrap().fill(0x6d);
        panic!("heap body unwind");
    }
}

#[ktest::suite(fixture = PressureFixture, tags("mm", "lifecycle", "pressure"), isolation = "Case", timeout_ms = 30000)]
mod pressure {
    use super::*;
    /// 测试目标：验证在耗尽全系统物理内存（空闲页为 0）的极端 OOM 状态下触发无堆分配的静态字符串 `panic!` 时，`PressureFixture` 能够安全回收全部压力页。
    ///
    /// 场景步骤（Given-When-Then）：
    /// 1. Given：由 `PressureFixture::set_up` 记录基线并预先分配好 `pressure` 句柄向量容量。
    /// 2. When：调用 `f.exhaust()` 分配并持有系统全部可用物理页直至空闲页为 0，随后主动触发静态字面量 `panic!("pressure body unwind")`。
    /// 3. Then：栈展开过程无需额外申请物理页，捕获后执行 `PressureFixture::tear_down` 释放全部压力页与句柄向量，系统内存统计完全恢复至基线。
    #[test]
    #[should_panic(expected = "pressure body unwind")]
    fn panic_keeps_owner(_: &mut TestContext<'_>, f: &mut PressureFixture) {
        f.exhaust();
        assert!(!f.pressure.is_empty());
        panic!("pressure body unwind");
    }
}
