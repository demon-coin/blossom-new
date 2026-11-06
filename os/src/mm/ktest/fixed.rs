//! 绑定固定 CPU 核（CPU 0）执行的内存测试夹具包装器与所有权安全交接机制。
//!
//! ## 模块职责与架构定位
//! 本模块位于 `mm::ktest` 下，为需要在固定硬件线程（CPU 0）上测试每 CPU 页缓存（PCP）或本地 SLUB 缓存的用例提供夹具封装（`FixedPages`、`FixedSlabs`、`FixedHeap`、`FixedPressure`）。
//! 由于 `ktest::spawn_on` 要求任务闭包满足 `'static` 生命周期，无法直接跨线程借用外层夹具，本模块通过 `Arc<SpinLock<Option<T>>>` 与 Drop 守卫 `Returned<T>` 实现夹具状态 `T` 的移动交接与异常展开自动归还。
//!
//! ## 核心机制：固定核夹具所有权流转
//! ```text
//! [控制线程 set_up]
//!    |-- 初始化内层夹具 T (记录 baseline)
//!    +-- 存入 Arc<SpinLock<Option<T>>> 共享槽位 (Some(T))
//!          |
//!          v
//! [run(body) -> spawn_on(CPU 0)]
//!    |-- CPU 0 子任务 lock().take() 取出 T，包装进 Returned<T> 守卫
//!    |-- 执行 body(&mut T)
//!    |-- 正常返回或 panic! 栈展开时，Returned::drop 自动将 T 放回共享槽位
//!    +-- 控制线程轮询 try_join() 等待子任务彻底退出内核栈
//!          |
//!          v
//! [控制线程 tear_down]
//!    |-- 从共享槽位取出归还的 T，释放 Arc<SpinLock<...>> 自身堆内存
//!    +-- 调用内层 <T as Fixture>::tear_down 释放全部测试资源并核对 baseline
//! ```
//!
//! ## 锁获取顺序与并发契约
//! - `Arc<SpinLock<Option<T>>>` 仅在子任务启动瞬间（`take`）与退出瞬间（`Returned::drop` 回填）加锁，持锁期间不执行任何内存分配或阻塞操作。
//! - 控制线程必须在 `execute` 中等待 `handle.try_join()` 成功后才会进入 `tear_down`，因此控制线程与 CPU 0 子任务对槽位的访问在时间上严格先后有序。

use crate::mm::ktest::{HeapFixture, PageFixture, PressureFixture, SlabFixture};
use crate::sync::SpinLock;
use alloc::sync::Arc;
use ktest::{Fixture, TestContext, TestError, TestResult};

/// 固定核子任务栈上的 RAII 归还守卫，确保无论子任务正常返回还是触发 `panic!` 栈展开，都能将夹具状态 `T` 移回共享槽位。
struct Returned<T> {
    /// 子任务执行期间独占持有的内层夹具实例（`Drop` 时通过 `take()` 移出）。
    state: Option<T>,
    /// 指向外层固定核夹具的共享回填槽位。
    destination: Arc<SpinLock<Option<T>>>,
}
impl<T> Drop for Returned<T> {
    /// 析构时获取自旋锁并将内层夹具状态 `state` 移回共享槽位 `destination`。
    fn drop(&mut self) {
        *self.destination.lock() = self.state.take();
    }
}

/// 将共享槽位 `state` 中的独占夹具状态移动到绑定在 CPU 0 上的子任务中执行 `body`，并在 20 秒超时窗口内等待子任务退出后归还状态。
fn execute<T: Send + 'static>(
    state: &Arc<SpinLock<Option<T>>>,
    body: impl FnOnce(&mut T) + Send + 'static,
) -> TestResult {
    let destination = Arc::clone(state);
    let handle = crate::ktest::spawn_on(0, move |_| {
        let state = destination
            .lock()
            .take()
            .expect("fixed CPU state already taken");
        let mut returned = Returned {
            state: Some(state),
            destination,
        };
        body(returned.state.as_mut().unwrap());
        Ok(())
    })?;

    let deadline = crate::arch::read_counter().wrapping_add(crate::arch::counter_frequency() * 20);
    while handle.try_join().is_none() {
        if deadline <= crate::arch::read_counter() {
            return Err(TestError("fixed CPU child did not leave stack"));
        }
        crate::sched::yield_now();
    }
    Ok(())
}

/// 生成绑定 CPU 0 执行的测试夹具包装结构体及其 `Fixture` 实现的宏。
macro_rules! fixed_fixture {
    ($name:ident, $state:ty) => {
        /// 将内层夹具 `$state` 绑定到 CPU 0 上执行的包装夹具。
        pub(crate) struct $name {
            /// 用于在控制线程与 CPU 0 子任务之间移动内层夹具所有权的共享自旋锁槽位。
            state: Arc<SpinLock<Option<$state>>>,
        }
        impl Fixture for $name {
            /// 初始化夹具：先构造内层夹具并记录内存基线，再分配 `Arc<SpinLock<...>>` 交接槽位。
            fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
                let state = <$state as Fixture>::set_up(context, &())?;
                let state = Arc::try_new(SpinLock::new(Some(state)))
                    .map_err(|_| TestError("NoMemory: fixed CPU control"))?;
                Ok(Self { state })
            }
            /// 清理夹具：在 CPU 0 子任务退出后取回内层夹具所有权，先行释放 `self` 的 `Arc` 控制块，再执行内层夹具的 `tear_down` 核对内存基线。
            fn tear_down(self, context: &mut TestContext<'_>) -> TestResult {
                let state = self
                    .state
                    .lock()
                    .take()
                    .ok_or(TestError("fixed CPU child retained owner"))?;
                drop(self);
                <$state as Fixture>::tear_down(state, context)
            }
        }
        impl $name {
            /// 将内层夹具移动至 CPU 0 子任务中执行测试闭包 `body`，等待其执行完毕并归还所有权。
            pub(crate) fn run(
                &mut self,
                body: impl FnOnce(&mut $state) + Send + 'static,
            ) -> TestResult {
                execute(&self.state, body)
            }
        }
    };
}
fixed_fixture!(FixedPages, PageFixture);
fixed_fixture!(FixedSlabs, SlabFixture);
fixed_fixture!(FixedHeap, HeapFixture);
fixed_fixture!(FixedPressure, PressureFixture);
