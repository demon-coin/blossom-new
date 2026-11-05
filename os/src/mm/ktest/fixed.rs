//! 固定核原 Task 的拥有权交接；控制者不借用 fixture 给静态入口。

use crate::mm::ktest::{HeapFixture, PageFixture, PressureFixture, SlabFixture};
use crate::sync::SpinLock;
use alloc::sync::Arc;
use ktest::{Fixture, TestContext, TestError, TestResult};

struct Returned<T> {
    state: Option<T>,
    destination: Arc<SpinLock<Option<T>>>,
}
impl<T> Drop for Returned<T> {
    /// 正常返回与 child panic 都把原 owner 移回 fixture，清理前框架仍须真实 join。
    fn drop(&mut self) {
        *self.destination.lock() = self.state.take();
    }
}

/// 将独占具体状态移动给固定 CPU0 原 Task，返回 guard 在展开时同样归还拥有权。
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

macro_rules! fixed_fixture {
    ($name:ident, $state:ty) => {
        pub(crate) struct $name {
            state: Arc<SpinLock<Option<$state>>>,
        }
        impl Fixture for $name {
            /// 预备具体 owner 与交接存储；原 Task 尚未发表，部分失败按 Rust Drop 清理。
            fn set_up(context: &mut TestContext<'_>, _: &()) -> Result<Self, TestError> {
                let state = <$state as Fixture>::set_up(context, &())?;
                let state = Arc::try_new(SpinLock::new(Some(state)))
                    .map_err(|_| TestError("NoMemory: fixed CPU control"))?;
                Ok(Self { state })
            }
            /// 框架已 join 原 child，才取回具体 owner；释放交接控制后按原六域基线核账。
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
            /// 正文只发表拥有状态的固定核任务，不发送上下文或 fixture 借用。
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
