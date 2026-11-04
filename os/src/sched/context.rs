//! 普通函数调用边界上的整数现场；CPU 基址始终由目的 CPU 保留。

#[cfg(target_arch = "riscv64")]
const SAVED_REGISTERS: usize = 12;
#[cfg(target_arch = "loongarch64")]
const SAVED_REGISTERS: usize = 10;

#[repr(C)]
pub struct TaskContext {
    pub(crate) ra: usize,
    pub(crate) sp: usize,
    pub(crate) saved: [usize; SAVED_REGISTERS],
}

impl TaskContext {
    /// 静态调度栈第一次切出时由汇编填入完整现场。
    pub const fn empty() -> Self {
        Self {
            ra: 0,
            sp: 0,
            saved: [0; SAVED_REGISTERS],
        }
    }

    /// 从独占栈顶开始执行入口，后续切换按普通 ABI 保存现场。
    pub(crate) fn initial(stack_top: usize) -> Self {
        assert_eq!(
            stack_top & (crate::arch::STACK_ALIGNMENT - 1),
            0,
            "task stack is not aligned"
        );
        Self {
            ra: crate::sched::task_trampoline as usize,
            sp: stack_top,
            saved: [0; SAVED_REGISTERS],
        }
    }
}
